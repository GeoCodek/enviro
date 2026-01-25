#!/usr/bin/env python3
"""
Enviro+ logger with GPS via gpsd (no PPS, no button)

Control:
- 3 proximity "near" events within 10 seconds toggle recording
- Optional: RECORD_ON_START=1

Display:
- RECORDING / IDLE
- GPS: WRITE (recording + fix)
- GPS: NO FIX (recording + no fix)
- GPS: IDLE (not recording)

CSV:
- Filename includes YYYY-MM-DD + unix timestamp
- Enviro+ fields + GPS fields (from gpsd TPV)
"""

import csv
import json
import os
import socket
import sys
import time
from collections import deque
from datetime import datetime

from enviroplus import gas, noise
from smbus2 import SMBus
# --- BME280 import (robust across packaging variants) ---
try:
    # classic Pimoroni layout expected by many Enviro+ examples
    from bme280 import BME280  # type: ignore
except Exception:
    try:
        # some installs expose a different module name
        from pimoroni_bme280 import BME280  # type: ignore
    except Exception:
        try:
            # last resort: package folder might be named "pimoroni_bme280"
            import importlib
            _m = importlib.import_module("pimoroni_bme280")
            BME280 = getattr(_m, "BME280")
        except Exception as e:
            raise ImportError(
                "Cannot import BME280. Your installed pimoroni-bme280 does not expose "
                "a compatible module name. Run the diagnostics (ls/pkgutil/grep) to see the real module."
            ) from e

# LTR559 (proximity + light)
try:
    from ltr559 import LTR559
    ltr559 = LTR559()
except Exception:
    import ltr559  # type: ignore

# Optional display
try:
    import st7735  # type: ignore
    from fonts.ttf import RobotoMedium as UserFont  # type: ignore
    from PIL import Image, ImageDraw, ImageFont  # type: ignore
except Exception:
    st7735 = None

# --------- Tuning parameters ---------
SAMPLE_INTERVAL_S = 5.0

GESTURE_WINDOW_S = 10.0
GESTURE_COUNT = 3

PROX_ON = 500
PROX_OFF = 150

TEMP_COMP_FACTOR = 2.25

GPSD_HOST = "127.0.0.1"
GPSD_PORT = 2947
GPSD_TIMEOUT_S = 0.25
# -------------------------------------


def now_iso_seconds() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def today_ymd() -> str:
    return datetime.now().strftime("%Y-%m-%d")


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


def get_cpu_temperature():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            return int(f.read()) / 1000.0
    except Exception:
        return None


def _truthy_env(name: str) -> bool:
    v = os.environ.get(name, "")
    return v.strip().lower() in ("1", "true", "yes", "y", "on")


def ensure_data_dir(script_dir: str) -> str:
    data_dir = os.path.join(script_dir, "data")
    os.makedirs(data_dir, exist_ok=True)
    return data_dir


def new_csv_path(data_dir: str) -> str:
    ts = int(time.time())
    return os.path.join(data_dir, f"enviro_log_{today_ymd()}_{ts}.csv")


# ---------- Display ----------
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
    img = Image.new("RGB", (display.width, display.height), color=(0, 0, 0))
    draw = ImageDraw.Draw(img)
    font = ImageFont.truetype(UserFont, 18)
    return {"display": display, "img": img, "draw": draw, "font": font, "w": display.width, "h": display.height}


def update_display_status(display_ctx, is_recording: bool, gps_state: str):
    """
    gps_state: "WRITE" | "NO FIX" | "IDLE" | "NO GPSD"
    """
    if not display_ctx:
        return
    draw = display_ctx["draw"]
    img = display_ctx["img"]
    font = display_ctx["font"]
    w = display_ctx["w"]
    h = display_ctx["h"]
    display = display_ctx["display"]

    draw.rectangle((0, 0, w, h), (0, 0, 0))

    draw.text((0, 0), "ENV LOG", font=font, fill=(255, 255, 255))

    status = "RECORDING" if is_recording else "IDLE"
    status_color = (0, 255, 0) if is_recording else (255, 0, 0)
    draw.text((0, 26), status, font=font, fill=status_color)

    # GPS line
    if gps_state == "WRITE":
        gps_color = (0, 200, 255)
    elif gps_state == "NO FIX":
        gps_color = (220, 220, 220)
    elif gps_state == "IDLE":
        gps_color = (180, 180, 180)
    else:  # NO GPSD / error
        gps_color = (255, 180, 0)

    draw.text((0, 54), f"GPS: {gps_state}", font=font, fill=gps_color)

    display.display(img)


# ---------- GPS via gpsd ----------
def gpsd_read_tpv(timeout=GPSD_TIMEOUT_S):
    """
    Returns dict with TPV fields or None if not available.
    Minimal watch -> read a few JSON lines, keep latest TPV.
    """
    try:
        with socket.create_connection((GPSD_HOST, GPSD_PORT), timeout=timeout) as s:
            s.settimeout(timeout)
            # enable JSON watch
            s.sendall(b'?WATCH={"enable":true,"json":true};\n')
            buf = b""
            latest_tpv = None
            t_end = time.time() + timeout
            while time.time() < t_end:
                try:
                    chunk = s.recv(4096)
                    if not chunk:
                        break
                    buf += chunk
                    # gpsd sends newline separated JSON
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        if not line.strip():
                            continue
                        try:
                            msg = json.loads(line.decode(errors="ignore"))
                        except Exception:
                            continue
                        if msg.get("class") == "TPV":
                            latest_tpv = msg
                except socket.timeout:
                    break
            return latest_tpv
    except Exception:
        return None


def tpv_to_fields(tpv):
    """
    Maps gpsd TPV to stable CSV fields.
    mode: 0/1/2/3 (>=2 means fix)
    """
    if not tpv:
        return {
            "mode": None, "lat": None, "lon": None, "alt": None,
            "speed": None, "track": None, "climb": None,
            "eph": None, "epv": None, "gpstime": None,
        }
    return {
        "mode": tpv.get("mode"),
        "lat": tpv.get("lat"),
        "lon": tpv.get("lon"),
        "alt": tpv.get("alt"),
        "speed": tpv.get("speed"),
        "track": tpv.get("track"),
        "climb": tpv.get("climb"),
        "eph": tpv.get("eph"),
        "epv": tpv.get("epv"),
        "gpstime": tpv.get("time"),
    }


# ---------- Sensors ----------
def init_sensors():
    env_noise = noise.Noise()
    bus = SMBus(1)
    bme = BME280(i2c_dev=bus)
    return env_noise, bme


def read_enviro(env_noise, bme):
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

    cpu = get_cpu_temperature()
    if temp is not None and cpu is not None:
        temp_comp = temp - ((cpu - temp) / TEMP_COMP_FACTOR)
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

    return {
        "lux": lux, "prox": prox,
        "temp": temp, "hum": hum, "pres": pres, "alt": alt,
        "cpu": cpu, "temp_comp": temp_comp,
        "ox": ox, "red": red, "nh3": nh3, "adc": adc,
        "nlow": nlow, "nmid": nmid, "nhigh": nhigh, "ntotal": ntotal,
    }


CSV_HEADER = [
    # meta
    "timestamp_iso",
    "unix_time_s",
    # enviro+
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
    # gps (via gpsd)
    "gps_mode",
    "gps_lat",
    "gps_lon",
    "gps_alt_m",
    "gps_speed_m_s",
    "gps_track_deg",
    "gps_climb_m_s",
    "gps_eph_m",
    "gps_epv_m",
    "gps_time_utc",
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


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = ensure_data_dir(script_dir)

    env_noise, bme = init_sensors()
    display_ctx = init_display()

    recorder = Recorder(data_dir)

    # gesture state
    events = deque()
    close_state = False

    next_sample = time.time()
    last_display_update = 0.0

    if _truthy_env("RECORD_ON_START"):
        recorder.start()

    print(f"[{now_iso_seconds()}] Ready. Gesture: {GESTURE_COUNT} proximity detections within {GESTURE_WINDOW_S}s toggles recording.")
    print(f"[{now_iso_seconds()}] Data directory: {data_dir}")

    try:
        while True:
            # Proximity for gesture detection
            try:
                prox_val = ltr559.get_proximity()
            except Exception:
                prox_val = 0

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

            now = time.time()

            # GPS status (for display + for CSV)
            tpv = gpsd_read_tpv()
            gps = tpv_to_fields(tpv)
            gps_fix = (gps["mode"] is not None and gps["mode"] >= 2)

            if recorder.is_recording:
                gps_state = "WRITE" if gps_fix else ("NO FIX" if tpv is not None else "NO GPSD")
            else:
                gps_state = "IDLE"

            # Display update ~1 Hz
            if display_ctx and (now - last_display_update) >= 1.0:
                update_display_status(display_ctx, recorder.is_recording, gps_state)
                last_display_update = now

            # Sampling
            if recorder.is_recording and now >= next_sample:
                e = read_enviro(env_noise, bme)

                row = [
                    now_iso_seconds(),
                    f"{now:.3f}",
                    safe_float(e["lux"]),
                    safe_float(e["prox"]),
                    safe_float(e["temp"]),
                    safe_float(e["hum"]),
                    safe_float(e["pres"]),
                    safe_float(e["alt"]),
                    safe_float(e["cpu"]),
                    safe_float(e["temp_comp"]),
                    safe_float(e["ox"]),
                    safe_float(e["red"]),
                    safe_float(e["nh3"]),
                    safe_float(e["adc"]),
                    safe_float(e["nlow"]),
                    safe_float(e["nmid"]),
                    safe_float(e["nhigh"]),
                    safe_float(e["ntotal"]),
                    safe_int(gps["mode"]),
                    safe_float(gps["lat"]),
                    safe_float(gps["lon"]),
                    safe_float(gps["alt"]),
                    safe_float(gps["speed"]),
                    safe_float(gps["track"]),
                    safe_float(gps["climb"]),
                    safe_float(gps["eph"]),
                    safe_float(gps["epv"]),
                    gps["gpstime"] if gps["gpstime"] is not None else "",
                ]

                recorder.write_row(row)
                next_sample = now + SAMPLE_INTERVAL_S

            time.sleep(0.05)

    except KeyboardInterrupt:
        pass
    finally:
        recorder.stop()
        print(f"[{now_iso_seconds()}] Exited.")


if __name__ == "__main__":
    main()
