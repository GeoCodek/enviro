#!/usr/bin/env python3
"""
Enviro+ controlled shutdown gesture.

Sequence:
1) Make 3 distinct "hand near" proximity events within 10 seconds.
2) Then make 3 strong lux-change events within 10 seconds.

When both stages complete in order, the script will issue a shutdown.
"""

import subprocess
import time
from collections import deque
from datetime import datetime

try:
    # Transitional fix for breaking change in LTR559
    from ltr559 import LTR559
    ltr559 = LTR559()
except ImportError:
    import ltr559


# --------- Tuning parameters ---------
GESTURE_WINDOW_S = 10.0
GESTURE_COUNT = 3

# Proximity thresholds (hysteresis)
PROX_ON = 500
PROX_OFF = 150

# Lux-change thresholds (hysteresis)
LUX_DELTA_ON = 40.0
LUX_DELTA_OFF = 20.0
LUX_BASELINE_ALPHA = 0.2
# -------------------------------------


def now_iso_seconds() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def shutdown():
    print(f"[{now_iso_seconds()}] SHUTDOWN triggered. Powering down...")
    subprocess.run(["sudo", "shutdown", "-h", "now"], check=False)


def main():
    stage = "prox"
    prox_events = deque()
    lux_events = deque()
    close_state = False
    wave_state = False
    lux_baseline = None

    print(
        f"[{now_iso_seconds()}] Ready. "
        f"Stage 1: {GESTURE_COUNT} proximity events within {GESTURE_WINDOW_S}s. "
        f"Stage 2: {GESTURE_COUNT} lux-change events within {GESTURE_WINDOW_S}s."
    )

    while True:
        now = time.time()
        if stage == "prox":
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
                    prox_events.append(now)

            while prox_events and (now - prox_events[0]) > GESTURE_WINDOW_S:
                prox_events.popleft()

            if len(prox_events) >= GESTURE_COUNT:
                prox_events.clear()
                stage = "lux"
                lux_events.clear()
                wave_state = False
                lux_baseline = None
                print(
                    f"[{now_iso_seconds()}] Stage 1 complete. "
                    f"Stage 2: wave over the lux sensor {GESTURE_COUNT} times within {GESTURE_WINDOW_S}s."
                )
                time.sleep(0.5)

        else:
            try:
                lux_val = ltr559.get_lux()
            except Exception:
                lux_val = None

            if lux_val is not None:
                if lux_baseline is None:
                    lux_baseline = lux_val

                if not wave_state:
                    lux_baseline = (
                        (1.0 - LUX_BASELINE_ALPHA) * lux_baseline
                        + LUX_BASELINE_ALPHA * lux_val
                    )

                lux_delta = abs(lux_val - lux_baseline)

                if wave_state:
                    if lux_delta <= LUX_DELTA_OFF:
                        wave_state = False
                else:
                    if lux_delta >= LUX_DELTA_ON:
                        wave_state = True
                        lux_events.append(now)

            while lux_events and (now - lux_events[0]) > GESTURE_WINDOW_S:
                lux_events.popleft()

            if len(lux_events) >= GESTURE_COUNT:
                shutdown()
                break

        time.sleep(0.05)


if __name__ == "__main__":
    main()
