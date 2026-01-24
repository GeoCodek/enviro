#!/usr/bin/env python3
"""
Enviro+ logger with GPS via gpsd (/dev/ttyACM0), proximity gesture toggle, and optional shutdown-watcher.

Control:
- Make 3 distinct "hand near" proximity events within 10 seconds to TOGGLE recording.
- Optional bypass: start recording immediately by setting RECORD_ON_START=1 when launching
  (or create a file named RECORD in the script directory).
  * If not recording: start logging to a new CSV file (with headers)
  * If recording: stop logging (close the CSV file)

GPS (USB via gpsd):
- Uses gpsd as the single owner of the GPS device (e.g. /dev/ttyACM0).
- This script reads position/time/quality from gpsd (no serial port contention).
- Robust: tolerates NO FIX, missing fields, and temporary gpsd outages.

Requirement (once):
- sudo apt -y install python3-gps

Notes:
- Designed to run inside Pimoroni's recommended venv: source ~/.virtualenvs/pimoroni/bin/activate
- If an ST7735 LCD is available, the display shows the current recording status.
- PMS5003 particulate logging is supported if the library is installed and the sensor can be read.
  If PMS reading fails, PM fields are left blank (no crash).
"""

import csv
import os
import subprocess
import sys
import time
from collections import deque
from datetime import datetime

# gpsd client (Debian package: python3-gps)
try:
    import gps  # type: ignore
    import select
except Exception:
    gps = None
    select = None

from enviroplus import gas, noise
from smbus2 import SMBus
from bme280 import BME280

# Optional PMS5003 support
try:
    from pms5003 import PMS5003  # type: ignore
except Exception:
    PMS5003 = None  # type: ignore

try:
    # Transitional fix for breaking change in LTR559
    from ltr559 import LTR559
    ltr559 = LTR559()
except ImportError:
    import ltr559  # type: ignore

try:
    import st7735  # type: ignore
    from fonts.ttf import RobotoMedium as UserFont  # type: ignore
    from PIL import Image, ImageDraw, ImageFont  # type: ignore
except ImportError:
    st7735 = None


def _truthy_env(name: str) -> bool:
    v = os.environ.get(name, "")
    return v.strip().lower() in ("1", "true", "yes", "y", "on")


def _record_flag_file(script_dir: str) -> str:
    return os.path.join(script_dir, "RECORD")


# --------- Tuning parameters ---------
SAMPLE_INTERVAL_S = 5.0

# Proximity gesture detection: "3 detections within 10 seconds"
GESTURE_WINDOW_S = 10.0
GESTURE_COUNT = 3

# Proximity thresholds (hysteresis). Adjust if needed.
PROX_ON = 500   # counts as "near" when rising above this
PROX_OFF = 150  # considered "far" again once dropping below this

# Temperature compensation factor (from Enviro+ examples)
TEMP_COMP_FACTOR = 2.25

# GPSD polling
GPSD_HOST = "127.0.0.1"
GPSD_PORT = "2947"
GPSD_MAX_MSGS_PER_LOOP = 25
# -------------------------------------


def now_iso_seconds() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def safe_float(x):
    try:
        if x is None or x == "":
            return ""
        return float(x)
    except Exception:
        return ""


def safe_int(x):
    try:
        if x is None or x == "":
            return ""
        return int(x)
    except Exception:
        return ""


def safe_str(x):
    if x is None:
        return ""
    try:
        return str(x)
    except Exception:
        return ""


def get_cpu_temperature():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            temp = f.read()
            return int(temp) / 1000.0
    except Exception:
        return None


def init_display():
    if st7735 is None:
        return None
    display = st7735.ST7735(
        port=0,
        cs=1,
        dc="GPIO9",
        backlight="GPIO12",
        rotation=270,
        spi_speed_hz=10000000,
    )
    display.begin()
    width = display.width
    height = display.height
    img = Image.new("RGB", (width, height), color=(0, 0, 0))
    draw = ImageDraw.Draw(img)
    font = ImageFont.truetype(UserFont, 20)
    return {
        "display": display,
        "width": width,
        "height": height,
        "img": img,
        "draw": draw,
        "font": font,
    }


def update_display_status(display_ctx, is_recording: bool):
    if not display_ctx:
        return
    draw = display_ctx["draw"]
    img = display_ctx["img"]
    width = display_ctx["width"]
    height = display_ctx["height"]
    font = display_ctx["font"]
    display = display_ctx["display"]
    draw.rectangle((0, 0, width, height), (0, 0, 0))
    status = "RECORDING" if is_recording else "IDLE"
    status_color = (0, 255, 0) if is_recording else (255, 0, 0)
    draw.text((0, 0), "READY", font=font, fill=(255, 255, 255))
    draw.text((0, 26), status, font=font, fill=status_color)
    display.display(img)


def init_sensors():
    env_noise = noise.Noise()
    bus = SMBus(1)
    bme = BME280(i2c_dev=bus)

    pms = None
    if PMS5003 is not None:
        try:
            pms = PMS5003()
        except Exception:
            pms = None

    return env_noise, bme, pms


def ensure_data_dir(script_dir: str) -> str:
    data_dir = os.path.join(script_dir, "data")
    os.makedirs(data_dir, exist_ok=True)
    return data_dir


def new_csv_path(data_dir: str) -> str:
    ts = int(time.time())
    return os.path.join(data_dir, f"enviro_log_{ts}.csv")


def start_shutdown_watcher(script_path: str, proc):
    if proc is not None:
        return proc
    if not os.path.exists(script_path):
        print(f"[{now_iso_seconds()}] Shutdown watcher script not found at {script_path}")
        return None
    print(f"[{now_iso_seconds()}] Starting shutdown watcher.")
    return subprocess.Popen([sys.executable, script_path])


def stop_shutdown_watcher(proc):
    if proc is None:
        return None
    print(f"[{now_iso_seconds()}] Stopping shutdown watcher.")
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except Exception:
        proc.kill()
    return None


class GPSState:
    """
    Snapshot state (always "latest we have from gpsd").
    When writing a row, we just take the current snapshot -> effectively "closest timestamp".
    """
    def __init__(self):
        self.lat = None
        self.lon = None
        self.alt_m = None
        self.speed_m_s = None
        self.track_deg = None

        self.mode = None  # 0/1/2/3 (no fix, 2D, 3D)
        self.hdop = None
        self.vdop = None
        self.pdop = None

        self.num_sats_visible = None
        self.num_sats_used = None

        self.gps_time_iso = None  # gpsd TPV time (UTC ISO8601) if present
        self.last_update_unix = None  # system time when we updated from gpsd

    def mark_updated(self):
        self.last_update_unix = time.time()


class GPSDClient:
    def __init__(self, host: str, port: str):
        self.host = host
        self.port = port
        self.session = None
        self.ok = False
        self.last_err = None

    def connect(self):
        if gps is None or select is None:
            self.last_err = "python3-gps not available (import gps failed)"
            self.ok = False
            return False
        try:
            s = gps.gps(host=self.host, port=self.port)
            s.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)
            self.session = s
            self.ok = True
            self.last_err = None
            return True
        except Exception as e:
            self.session = None
            self.ok = False
            self.last_err = str(e)
            return False

    def poll_into(self, state: GPSState, max_msgs: int = 25):
        """
        Non-blocking poll: read up to max_msgs waiting reports from gpsd and update state.
        If connection drops, we degrade silently and try reconnect occasionally.
        """
        if not self.ok or self.session is None:
            return

        # If gpsd is quiet, select() returns immediately.
        for _ in range(max_msgs):
            try:
                r, _, _ = select.select([self.session.sock], [], [], 0)
                if not r:
                    return

                report = self.session.next()
                if not isinstance(report, dict):
                    # python-gps sometimes returns objects; try dict-like fallback
                    try:
                        report = dict(report)
                    except Exception:
                        continue

                cls = report.get("class", "")
                if cls == "TPV":
                    # TPV: lat/lon/alt/speed/track/time/mode...
                    if "lat" in report:
                        state.lat = report.get("lat")
                    if "lon" in report:
                        state.lon = report.get("lon")
                    if "alt" in report:
                        state.alt_m = report.get("alt")
                    if "speed" in report:
                        state.speed_m_s = report.get("speed")  # m/s
                    if "track" in report:
                        state.track_deg = report.get("track")
                    if "mode" in report:
                        state.mode = report.get("mode")
                    if "time" in report:
                        state.gps_time_iso = report.get("time")

                    state.mark_updated()

                elif cls == "SKY":
                    # SKY: satellites list, used/visible counts, hdop/vdop/pdop...
                    sats = report.get("satellites", None)
                    if isinstance(sats, list):
                        state.num_sats_visible = len(sats)
                        used = 0
                        for s in sats:
                            try:
                                if s.get("used"):
                                    used += 1
                            except Exception:
                                pass
                        state.num_sats_used = used

                    if "hdop" in report:
                        state.hdop = report.get("hdop")
                    if "vdop" in report:
                        state.vdop = report.get("vdop")
                    if "pdop" in report:
                        state.pdop = report.get("pdop")

                    state.mark_updated()

            except StopIteration:
                # gpsd socket ended
                self.ok = False
                self.last_err = "gpsd stream ended"
                return
            except Exception as e:
                self.ok = False
                self.last_err = str(e)
                return


CSV_HEADER = [
    "timestamp_iso",
    "unix_time_s",

    # --- GPS (via gpsd) ---
    "gps_lat_deg",
    "gps_lon_deg",
    "gps_alt_m",
    "gps_speed_m_s",
    "gps_track_deg",
    "gps_mode",
    "gps_hdop",
    "gps_vdop",
    "gps_pdop",
    "gps_sats_visible",
    "gps_sats_used",
    "gps_time_iso",
    "gps_last_update_unix_s",

    # --- Enviro+ ---
    "lux",
    "proximity",
    "temperature_C",
    "humidity_percent",
    "pressure_hPa",
    "altitude_m",
    "cpu_temperature_C",
    "temperature_compensated_C",
    "gas_oxidising_ohms",
    "gas_reducing_ohms",
    "gas_nh3_ohms",
    "gas_adc_raw",
    "noise_low",
    "noise_mid",
    "noise_high",
    "noise_total",

    # PMS5003 (blank if unavailable)
    "pm1_0_ug_m3",
    "pm2_5_ug_m3",
    "pm10_ug_m3",
    "pm1_0_atm_ug_m3",
    "pm2_5_atm_ug_m3",
    "pm10_atm_ug_m3",
    "pm0_3_count",
    "pm0_5_count",
    "pm1_0_count",
    "pm2_5_count",
    "pm5_0_count",
    "pm10_count",
]


class Recorder:
    def __init__(self, data_dir: str):
        self.data_dir = data_dir
        self.fp = None
        self.writer = None
        self.path = None

    @property
    def is_recording(self) -> bool:
        return self.fp is not None

    def start(self):
        if self.is_recording:
            return
        self.path = new_csv_path(self.data_dir)
        self.fp = open(self.path, "w", newline="")
        self.writer = csv.writer(self.fp)
        self.writer.writerow(CSV_HEADER)
        self.fp.flush()
        print(f"[{now_iso_seconds()}] RECORDING STARTED -> {self.path}")

    def stop(self):
        if not self.is_recording:
            return
        try:
            self.fp.flush()
            self.fp.close()
        finally:
            print(f"[{now_iso_seconds()}] RECORDING STOPPED -> {self.path}")
            self.fp = None
            self.writer = None
            self.path = None

    def write_row(self, row):
        if not self.is_recording:
            return
        self.writer.writerow(row)
        self.fp.flush()


def _pms_value(pm, attr: str, size, atm_fallback: bool = False):
    x = getattr(pm, attr, None)
    if x is None:
        return None

    if callable(x):
        try:
            if atm_fallback:
                return x(size, True)
            return x(size)
        except TypeError:
            try:
                return x(size)
            except Exception:
                return None
        except Exception:
            return None

    try:
        if hasattr(x, "get"):
            return x.get(size, None)
        return x[size]
    except Exception:
        return None


def read_all(env_noise, bme, pms):
    # Light/proximity
    try:
        lux = ltr559.get_lux()
    except Exception:
        lux = None

    try:
        prox = ltr559.get_proximity()
    except Exception:
        prox = None

    # Weather
    try:
        temp = bme.get_temperature()
    except Exception:
        temp = None

    try:
        hum = bme.get_humidity()
    except Exception:
        hum = None

    try:
        pres = bme.get_pressure()
    except Exception:
        pres = None

    try:
        alt = bme.get_altitude()
    except Exception:
        alt = None

    # CPU temperature for compensation (if available)
    cpu_temp = get_cpu_temperature()
    if temp is not None and cpu_temp is not None:
        temp_comp = temp - ((cpu_temp - temp) / TEMP_COMP_FACTOR)
    else:
        temp_comp = None

    # Gas
    try:
        g = gas.read_all()
        ox = getattr(g, "oxidising", None)
        red = getattr(g, "reducing", None)
        nh3 = getattr(g, "nh3", None)
        adc = getattr(g, "adc", None)
    except Exception:
        ox = red = nh3 = adc = None

    # Noise
    try:
        nlow, nmid, nhigh, ntotal = env_noise.get_noise_profile()
    except Exception:
        nlow = nmid = nhigh = ntotal = None

    # PMS5003 (optional)
    pm1_0 = pm2_5 = pm10 = None
    pm1_0_atm = pm2_5_atm = pm10_atm = None
    pm0_3_count = pm0_5_count = pm1_0_count = None
    pm2_5_count = pm5_0_count = pm10_count = None

    if pms is not None:
        try:
            pm = pms.read()

            pm1_0 = _pms_value(pm, "pm_ug_per_m3", 1.0)
            pm2_5 = _pms_value(pm, "pm_ug_per_m3", 2.5)
            pm10 = _pms_value(pm, "pm_ug_per_m3", 10)

            pm1_0_atm = _pms_value(pm, "pm_ug_per_m3_atm", 1.0) or _pms_value(pm, "pm_ug_per_m3", 1.0, atm_fallback=True)
            pm2_5_atm = _pms_value(pm, "pm_ug_per_m3_atm", 2.5) or _pms_value(pm, "pm_ug_per_m3", 2.5, atm_fallback=True)
            pm10_atm = _pms_value(pm, "pm_ug_per_m3_atm", 10) or _pms_value(pm, "pm_ug_per_m3", 10, atm_fallback=True)

            pm0_3_count = _pms_value(pm, "pm_per_1l_air", 0.3)
            pm0_5_count = _pms_value(pm, "pm_per_1l_air", 0.5)
            pm1_0_count = _pms_value(pm, "pm_per_1l_air", 1.0)
            pm2_5_count = _pms_value(pm, "pm_per_1l_air", 2.5)
            pm5_0_count = _pms_value(pm, "pm_per_1l_air", 5.0)
            pm10_count = _pms_value(pm, "pm_per_1l_air", 10)
        except Exception:
            pass

    return {
        "lux": lux,
        "prox": prox,
        "temp": temp,
        "hum": hum,
        "pres": pres,
        "alt": alt,
        "cpu_temp": cpu_temp,
        "temp_comp": temp_comp,
        "ox": ox,
        "red": red,
        "nh3": nh3,
        "adc": adc,
        "nlow": nlow,
        "nmid": nmid,
        "nhigh": nhigh,
        "ntotal": ntotal,
        "pm1_0": pm1_0,
        "pm2_5": pm2_5,
        "pm10": pm10,
        "pm1_0_atm": pm1_0_atm,
        "pm2_5_atm": pm2_5_atm,
        "pm10_atm": pm10_atm,
        "pm0_3_count": pm0_3_count,
        "pm0_5_count": pm0_5_count,
        "pm1_0_count": pm1_0_count,
        "pm2_5_count": pm2_5_count,
        "pm5_0_count": pm5_0_count,
        "pm10_count": pm10_count,
    }


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = ensure_data_dir(script_dir)

    env_noise, bme, pms = init_sensors()
    display_ctx = init_display()

    shutdown_script = os.path.join(script_dir, "enviro_pi_shutdown_gesture.py")
    shutdown_proc = None

    recorder = Recorder(data_dir)

    # GPSD setup/state
    gps_state = GPSState()
    gpsd = GPSDClient(GPSD_HOST, GPSD_PORT)
    gpsd.connect()
    last_gpsd_reconnect_try = 0.0

    # Proximity gesture state
    events = deque()
    close_state = False

    next_sample = time.time()
    last_status = None

    print(
        f"[{now_iso_seconds()}] Ready. Gesture: {GESTURE_COUNT} proximity detections within "
        f"{GESTURE_WINDOW_S}s toggles recording."
    )
    print(f"[{now_iso_seconds()}] Data directory: {data_dir}")
    if gps is None:
        print(f"[{now_iso_seconds()}] GPSD: disabled (python3-gps missing). Install: sudo apt -y install python3-gps")
    else:
        print(f"[{now_iso_seconds()}] GPSD: {GPSD_HOST}:{GPSD_PORT} (connected={gpsd.ok})")

    # Optional bypass: start recording immediately if requested.
    if _truthy_env("RECORD_ON_START") or os.path.exists(_record_flag_file(script_dir)):
        recorder.start()
    else:
        shutdown_proc = start_shutdown_watcher(shutdown_script, shutdown_proc)

    last_status = recorder.is_recording
    print(f"[{now_iso_seconds()}] STATUS: {'RECORDING' if last_status else 'IDLE'}")
    update_display_status(display_ctx, last_status)

    try:
        while True:
            # Keep GPS snapshot fresh regardless of recording state.
            if gps is not None:
                if not gpsd.ok:
                    # Try reconnect every ~5s
                    now_r = time.time()
                    if (now_r - last_gpsd_reconnect_try) > 5.0:
                        last_gpsd_reconnect_try = now_r
                        gpsd.connect()
                if gpsd.ok:
                    gpsd.poll_into(gps_state, max_msgs=GPSD_MAX_MSGS_PER_LOOP)

            # Poll proximity frequently for gesture detection
            try:
                prox_val = ltr559.get_proximity()
            except Exception:
                prox_val = 0

            # Hysteresis close/far state
            if close_state:
                if prox_val <= PROX_OFF:
                    close_state = False
            else:
                if prox_val >= PROX_ON:
                    close_state = True
                    t = time.time()
                    events.append(t)
                    while events and (t - events[0]) > GESTURE_WINDOW_S:
                        events.popleft()
                    if len(events) >= GESTURE_COUNT:
                        events.clear()
                        if recorder.is_recording:
                            recorder.stop()
                        else:
                            recorder.start()
                        time.sleep(0.5)

            if last_status != recorder.is_recording:
                last_status = recorder.is_recording
                print(f"[{now_iso_seconds()}] STATUS: {'RECORDING' if last_status else 'IDLE'}")
                update_display_status(display_ctx, last_status)
                if last_status:
                    shutdown_proc = stop_shutdown_watcher(shutdown_proc)
                else:
                    shutdown_proc = start_shutdown_watcher(shutdown_script, shutdown_proc)

            # If recording, write sensor sample at fixed interval
            now = time.time()
            if recorder.is_recording and now >= next_sample:
                s = read_all(env_noise, bme, pms)

                row = [
                    now_iso_seconds(),
                    f"{now:.3f}",

                    # GPS snapshot from gpsd (blank if no fix / not yet available)
                    safe_float(gps_state.lat),
                    safe_float(gps_state.lon),
                    safe_float(gps_state.alt_m),
                    safe_float(gps_state.speed_m_s),
                    safe_float(gps_state.track_deg),
                    safe_int(gps_state.mode),
                    safe_float(gps_state.hdop),
                    safe_float(gps_state.vdop),
                    safe_float(gps_state.pdop),
                    safe_int(gps_state.num_sats_visible),
                    safe_int(gps_state.num_sats_used),
                    safe_str(gps_state.gps_time_iso),
                    safe_float(gps_state.last_update_unix),

                    # Enviro+ sensors
                    safe_float(s["lux"]),
                    safe_float(s["prox"]),
                    safe_float(s["temp"]),
                    safe_float(s["hum"]),
                    safe_float(s["pres"]),
                    safe_float(s["alt"]),
                    safe_float(s["cpu_temp"]),
                    safe_float(s["temp_comp"]),
                    safe_float(s["ox"]),
                    safe_float(s["red"]),
                    safe_float(s["nh3"]),
                    safe_float(s["adc"]),
                    safe_float(s["nlow"]),
                    safe_float(s["nmid"]),
                    safe_float(s["nhigh"]),
                    safe_float(s["ntotal"]),

                    # PMS5003
                    safe_float(s["pm1_0"]),
                    safe_float(s["pm2_5"]),
                    safe_float(s["pm10"]),
                    safe_float(s["pm1_0_atm"]),
                    safe_float(s["pm2_5_atm"]),
                    safe_float(s["pm10_atm"]),
                    safe_float(s["pm0_3_count"]),
                    safe_float(s["pm0_5_count"]),
                    safe_float(s["pm1_0_count"]),
                    safe_float(s["pm2_5_count"]),
                    safe_float(s["pm5_0_count"]),
                    safe_float(s["pm10_count"]),
                ]

                recorder.write_row(row)
                print(",".join(str(x) for x in row))
                next_sample = now + SAMPLE_INTERVAL_S

            time.sleep(0.05)

    except KeyboardInterrupt:
        pass
    finally:
        recorder.stop()
        shutdown_proc = stop_shutdown_watcher(shutdown_proc)
        print(f"[{now_iso_seconds()}] Exited.")


if __name__ == "__main__":
    main()
```
