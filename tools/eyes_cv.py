#!/usr/bin/env python3
"""sateq backlash oracle v2 — camera-mounted-on-the-mount, EDGE-based.

The camera rides the mount and looks at a static scene (room edge rich:
projection screen, shelves). Any axis motion shifts the whole frame.
Room light through windows changes slowly (shadows); brightness
differencing false-alarms on that, so we compare EDGE maps instead:

  edge = gray - gaussian(gray)        # high-pass: kills slow/soft lighting
  edge = edge / std(edge)             # kills overall gain/exposure shifts
  changed = pixels |edge_new - edge_base| > EYES_THR   (units of edge sigma)

Object edges (hard lines) survive the high-pass; window shadows are
low-frequency and largely vanish. When the mount moves, every object
edge in view shifts -> big changed-pixel counts.

Usage:
  eyes_cv.py peek [out.png]       save one frame (sanity check framing)
  eyes_cv.py reset                recapture baseline edge map
  eyes_cv.py query                exit 0 = moved, 1 = still, 2 = error
  eyes_cv.py watch [seconds]      human check; print changed-px each read

With eqtool:  eqtool ... backlash --eyes cmd --eyes-cmd /path/eyes_cv.py
(the wizard calls `reset` at each reversal and `query` after each burst;
export EYES_ROI in the shell so both sides agree). The camera's 3A must
be FROZEN for the baseline to be meaningful: manual focus and exposure
(v4l2-ctl: focus_automatic_continuous=0, auto_exposure=0), white balance
off, sharpness pinned — see AGENTS.md "BACKLASH MEASUREMENT".

Environment:
  EYES_DEVICE   video device            (default /dev/video0)
  EYES_ROI      x0:x1:y0:y1 fractional  (default whole frame; EXCLUDE any
                                         monitor screens — content changes)
  EYES_THR      edge-diff threshold     (default 2.5 sigma)
  EYES_PX       changed-px to call MOVED(default 300)
  EYES_SIGMA    high-pass sigma         (default 6 px)
  EYES_VERBOSE  1 = print px counts on query
Baseline lives in ~/.cache/sateq-eyes/base_<device>.npy and bakes in the
ROI in effect at `reset` time — after changing EYES_ROI, run `reset`
again before `query`/watch-compare (the eqtool wizard's per-burst
`reset` inherits EYES_ROI from its own environment, which is correct).
"""
import os
import sys
import time

import cv2
import numpy as np

DEV = os.environ.get("EYES_DEVICE", "/dev/video0")
X1, Y1 = 1920, 1080
BASE_DIR = os.path.expanduser("~/.cache/sateq-eyes")
BASE = os.path.join(BASE_DIR, f"base_{os.path.basename(DEV)}.npy")
THR = float(os.environ.get("EYES_THR", "2.5"))
MOVED_PX = int(os.environ.get("EYES_PX", "300"))
HP_SIGMA = float(os.environ.get("EYES_SIGMA", "6"))


def capture_open(quiet=False):
    cap = cv2.VideoCapture(DEV, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, X1)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, Y1)
    if not cap.isOpened():
        if not quiet:
            print(f"cannot open {DEV}", file=sys.stderr)
        sys.exit(2)
    good = 0
    deadline = time.monotonic() + 5.0
    while good < 10 and time.monotonic() < deadline:
        ok, f = cap.read()
        if ok and f is not None and f.size:
            good += 1
    if good < 10:
        cap.release()
        if not quiet:
            print("camera never delivered settling frames", file=sys.stderr)
        sys.exit(2)
    return cap


def _roi(frame):
    spec = os.environ.get("EYES_ROI", "")
    h, w = frame.shape[:2]
    if spec:
        f = [float(v) for v in spec.split(":")]
        if len(f) == 4 and f[1] > f[0] and f[3] > f[2] and min(f) >= 0:
            x0, x1, y0, y1 = f
            return int(x0 * w), int(x1 * w), int(y0 * h), int(y1 * h)
    return 0, w, 0, h


def _edge_map(gray):
    """High-pass, gain-normalized edge response (float32)."""
    blur = cv2.GaussianBlur(gray, (0, 0), HP_SIGMA)
    hp = gray.astype(np.float32) - blur.astype(np.float32)
    s = float(hp.std())
    if s < 1e-6:
        s = 1.0
    return hp / np.float32(s)


def _capture(cap):
    r = capture_open() if cap is None else cap
    frame = None
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        ok, f = r.read()
        if ok and f is not None and f.size:
            frame = f
            break
        time.sleep(0.05)
    if frame is None:
        print("read failed", file=sys.stderr)
        sys.exit(2)
    if cap is None:
        r.release()
    x0, x1, y0, y1 = _roi(frame)
    return _edge_map(cv2.cvtColor(frame[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY))


def _count(r):
    return (np.abs(r) > THR).sum()


def _deltas(a, b):
    return np.abs(a - b)


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else ""
    if mode == "peek":
        out = sys.argv[2] if len(sys.argv) > 2 else "peek.png"
        cap = capture_open()
        ok, frame = cap.read()
        cap.release()
        if not ok:
            sys.exit(2)
        cv2.imwrite(out, frame)
        print(out)
        return
    cap = None
    if mode == "reset":
        os.makedirs(BASE_DIR, exist_ok=True)
        np.save(BASE, _capture(None))
        return
    if mode == "query":
        base = np.load(BASE)
        cur = _capture(None)
        px = _count(_deltas(base, cur))
        if os.environ.get("EYES_VERBOSE"):
            print(px)
        sys.exit(1 if px < MOVED_PX else 0)
    if mode == "watch":
        secs = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
        cap = capture_open(quiet=True)
        base = _capture(cap)
        t0 = time.monotonic()
        print(f"baseline edge map captured; px = changed edge pixels "
              f"(thr {THR}, moved at {MOVED_PX}); watching {secs:.0f}s")
        while time.monotonic() - t0 < secs:
            px = int(_count(_deltas(base, _capture(cap))))
            print(f"t={time.monotonic() - t0:5.2f} px={px:6d}"
                  f" {'MOVED' if px >= MOVED_PX else 'still'}")
            time.sleep(0.2)
        cap.release()
        return
    print(__doc__, file=sys.stderr)
    sys.exit(2)


if __name__ == "__main__":
    main()
