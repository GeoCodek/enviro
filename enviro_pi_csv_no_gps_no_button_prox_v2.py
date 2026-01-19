#!/usr/bin/env python3
"""

Notes:
- Particulate sensor (PMS5003) is intentionally not used here to avoid blocking reads when not attached.
- Designed to run inside Pimoroni's recommended venv: source ~/.virtualenvs/pimoroni/bin/activate
- If an ST7735 LCD is available, the display shows the current recording status.

"""

import csv
import os
import subprocess
import sys
import time
from collections import deque
from datetime import datetime
from enviroplus import gas, noise
from smbus2 import SMBus
from bme280 import BME280
from pms5003 import PMS5003

try:
    # Transitional fix for breaking change in LTR559
    from ltr559 import LTR559
    ltr559 = LTR559()
except ImportError:
    import ltr559

try:
    import st7735
    from fonts.ttf import RobotoMedium as UserFont
    from PIL import Image, ImageDraw, ImageFont
except ImportError:
    st7735 = None
 

def _truthy_env(name: str) -> bool:
GESTURE_COUNT = 3

# Proximity thresholds (hysteresis). Adjust if needed.
PROX_ON = 500   # counts as "near" when rising above this
PROX_OFF = 150  # considered "far" again once dropping below this

# Temperature compensation factor (from Enviro+ examples)
TEMP_COMP_FACTOR = 2.25
# -------------------------------------

        return ""


def get_cpu_temperature():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            temp = f.read()
            return int(temp) / 1000.0
    except Exception:
        return ""


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
    pms = PMS5003()
    return env_noise, bme, pms


def ensure_data_dir(script_dir: str) -> str:

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


CSV_HEADER = [
    "timestamp_iso",
    "unix_time_s",
    "humidity_percent",
    "pressure_hPa",
    "altitude_m",
    "cpu_temperature_C",
    "temperature_compensated_C",
    "gas_oxidising_ohms",
    "gas_reducing_ohms",
    "gas_nh3_ohms",
    "noise_mid",
    "noise_high",
    "noise_total",
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

        self.fp.flush()


def read_all(env_noise, bme, pms):
    # Light/proximity
    try:
        lux = ltr559.get_lux()

    except Exception:
        alt = ""

    # CPU temperature for compensation (if available)
    cpu_temp = get_cpu_temperature()
    if temp != "" and cpu_temp != "":
        temp_comp = temp - ((cpu_temp - temp) / TEMP_COMP_FACTOR)
    else:
        temp_comp = ""

    # Gas (resistance-like values)
    try:
        g = gas.read_all()

 
    except Exception:
        nlow = nmid = nhigh = ntotal = ""

    # Particulates (PMS5003)
    try:
        pm = pms.read()
        pm1_0 = getattr(pm, "pm_ug_per_m3", {}).get(1.0, "")
        pm2_5 = getattr(pm, "pm_ug_per_m3", {}).get(2.5, "")
        
 #The PMS5003 reading object exposes `pm_ug_per_m3(...)` as a callable method (see examples/combined.py lines 281–286), but here it’s treated like a dict and accessed via `.get`. If the sensor is connected, `getattr(pm, "pm_ug_per_m3", {})` returns a function, so calling `.get` raises `AttributeError` and the exception handler blanks every PM field. That means particulate columns stay empty even when the PMS5003 is present. This should call the methods directly (and similarly for the particle count accessor) to actually log PM data.
        pm10 = getattr(pm, "pm_ug_per_m3", {}).get(10, "")
        pm1_0_atm = getattr(pm, "pm_ug_per_m3_atm", {}).get(1.0, "")
        pm2_5_atm = getattr(pm, "pm_ug_per_m3_atm", {}).get(2.5, "")
        pm10_atm = getattr(pm, "pm_ug_per_m3_atm", {}).get(10, "")
        pm0_3_count = getattr(pm, "pm_per_1l_air", {}).get(0.3, "")
        pm0_5_count = getattr(pm, "pm_per_1l_air", {}).get(0.5, "")
        pm1_0_count = getattr(pm, "pm_per_1l_air", {}).get(1.0, "")
        pm2_5_count = getattr(pm, "pm_per_1l_air", {}).get(2.5, "")
        pm5_0_count = getattr(pm, "pm_per_1l_air", {}).get(5.0, "")
        pm10_count = getattr(pm, "pm_per_1l_air", {}).get(10, "")
    except Exception:
        pm1_0 = pm2_5 = pm10 = ""
        pm1_0_atm = pm2_5_atm = pm10_atm = ""
        pm0_3_count = pm0_5_count = pm1_0_count = ""
        pm2_5_count = pm5_0_count = pm10_count = ""

    return {
        "lux": lux,
        "prox": prox,
        "hum": hum,
        "pres": pres,
        "alt": alt,
        "cpu_temp": cpu_temp,
        "temp_comp": temp_comp,
        "ox": ox,
        "red": red,
        "nh3": nh3,
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

    close_state = False  # hysteresis state

    next_sample = time.time()
    last_status = None

    print(f"[{now_iso_seconds()}] Ready. Gesture: {GESTURE_COUNT} proximity detections within {GESTURE_WINDOW_S}s toggles recording.")
    print(f"[{now_iso_seconds()}] Data directory: {data_dir}")
 
    # - OR create a file named 'RECORD' in the script directory
    if _truthy_env("RECORD_ON_START") or os.path.exists(_record_flag_file(script_dir)):
        recorder.start()
    else:
        shutdown_proc = start_shutdown_watcher(shutdown_script, shutdown_proc)

    last_status = recorder.is_recording
    print(f"[{now_iso_seconds()}] STATUS: {'RECORDING' if last_status else 'IDLE'}")
    update_display_status(display_ctx, last_status)

    try:
        while True:
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
                 
                    safe_float(s["hum"]),
                    safe_float(s["pres"]),
                    safe_float(s["alt"]),
                    safe_float(s["cpu_temp"]),
                    safe_float(s["temp_comp"]),
                    safe_float(s["ox"]),
                    safe_float(s["red"]),
                    safe_float(s["nh3"]),
                    safe_float(s["nmid"]),
                    safe_float(s["nhigh"]),
                    safe_float(s["ntotal"]),
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
             
        pass
    finally:
        recorder.stop()
        shutdown_proc = stop_shutdown_watcher(shutdown_proc)
        print(f"[{now_iso_seconds()}] Exited.")



