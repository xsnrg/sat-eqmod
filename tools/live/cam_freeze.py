#!/usr/bin/env python3
"""Freeze the C930e for the eyes oracle: pick best focus by edge sharpness,
exposure by target brightness, then persist via v4l2-ctl."""
import re
import subprocess
import sys

import cv2
import numpy as np

DEV = "/dev/video0"
CAP_IDX = 0
W, H = 1280, 720


def ctl(**kw):
    args = ["v4l2-ctl", "-d", DEV, "-c",
            ",".join(f"{k}={v}" for k, v in kw.items())]
    subprocess.run(args, check=True, capture_output=True)


def get(name):
    out = subprocess.run(["v4l2-ctl", "-d", DEV, "-C", name],
                         capture_output=True, text=True).stdout
    m = re.search(r":\s*(\d+)", out)
    return int(m.group(1)) if m else -1


def grab(cap, n=4):
    acc = None
    for _ in range(n):
        ok, f = cap.read()
        if not ok:
            return None
        g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY).astype(np.float32)
        acc = g if acc is None else acc + g
    return acc / n


cap = cv2.VideoCapture(CAP_IDX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
cap.set(cv2.CAP_PROP_EXPOSURE, -4)  # temporary bright-ish baseline for focus

# 1) manual exposure + WB off so nothing hunts under us; sharpness pinned to
# the NEUTRAL default (leftover enhancement amplifies noise frame-to-frame)
ctl(auto_exposure=1, exposure_dynamic_framerate=0, white_balance_automatic=0,
    focus_automatic_continuous=0, power_line_frequency=0, sharpness=128)
ctl(exposure_time_absolute=250)  # 25 ms: light enough for a meaningful focus sweep

# 2) focus sweep: sharpness = variance of Laplacian. The C930e actuator is
# grossly nonlinear: the useful zone sits at the low end (measured plateau
# 0..32, collapse by ~48, featureless beyond), so coarse high-range sweeps
# are useless — hunt dense at low end first, only walk up if still climbing.
best_f, best_s = 0, -1
for f in list(range(0, 96, 4)) + [128, 192, 256, 512, 1023]:
    ctl(focus_absolute=f)
    g = grab(cap)
    if g is None:
        continue
    s = cv2.Laplacian(g, cv2.CV_32F).var()
    if s > best_s:
        best_s, best_f = s, f
# refine +/- one coarse step
lo, hi = max(0, best_f - 4), min(1023, best_f + 4)
for f in range(lo, hi + 1):
    ctl(focus_absolute=f)
    g = grab(cap)
    s = cv2.Laplacian(g, cv2.CV_32F).var() if g is not None else -1
    if s > best_s:
        best_s, best_f = s, f
ctl(focus_absolute=best_f)
cap.set(cv2.CAP_PROP_EXPOSURE, 0)  # stop our temp AE override influence

# 3) exposure sweep: mean gray closest to 110
best_e, best_err = 500, 1e9
for e in (100, 160, 250, 333, 500, 700, 1000, 1400, 2047):
    ctl(exposure_time_absolute=e)
    g = grab(cap)
    if g is None:
        continue
    err = abs(float(g.mean()) - 110.0)
    if err < best_err:
        best_err, best_e = err, e
ctl(exposure_time_absolute=best_e)

# 4) freeze gain at a good value for the chosen exposure
g = grab(cap)
ctl(white_balance_temperature=6500)
mean = float(g.mean()) if g is not None else -1
print(f"focus_absolute={best_f} sharpness={best_s:.0f} "
      f"exposure_time_absolute={best_e} mean_gray={mean:.0f}")
print("frozen:", get("focus_absolute"), get("exposure_time_absolute"),
      get("auto_exposure"), get("white_balance_automatic"),
      get("focus_automatic_continuous"))
cap.release()
sys.exit(0)
