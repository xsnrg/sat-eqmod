#!/usr/bin/env python3
"""Envelope creep probe: walk one axis past the guessed travel limit in
small increments, pausing for the operator's eyes each step. (C)2026 MIT,
see LICENSE.

Finds the REAL mechanical limit of each axis, replacing the guessed
+-105 deg datum-relative envelope. Deliberately dumb: it shells out to
`eqtool rate` (raw motor wind + :K stop, counter-verified) — no daemon,
no control law, no envelope in the loop. The port must be free: the
script REFUSES if sateqd is running (two owners of the port is forbidden).

Begin with the mount physically parked at home (level, tube due east,
tube axis level = the calibrated datum). The first run records the datum
counters to <state>/envelope-probe.json; every later step prints each
axis's signed degrees-from-datum, so the running envelope number you are
testing is always on screen.

Usage on the bench host:

  ./envelope_probe.py --axis ra  --dir -          # one 5-deg RA step west/up
  ./envelope_probe.py --axis ra  --dir +
  ./envelope_probe.py --axis dec --dir +
  ./envelope_probe.py --axis ra  --dir - --deg 2  # smaller steps near the wall
  ./envelope_probe.py --axis ra  --dir + --deg -5 # UNWIND 5 deg (negative dir moves back)
  ./envelope_probe.py --reset                     # re-datum at current pose

Tube paths from home (verified against astro.cpp AltAzLevel algebra):
  RA wing − : tube climbs the east sky, crosses near zenith, descends
              west; the counter arm winds the cable bundle this far.
  RA wing + : from home-dec this aims below the ground — only probe it
              after walking DEC up first (document the pose you used).
  DEC +     : tube walks the horizon east -> north -> west (shaft leads);
              past-zenital clearance side.
  DEC −     : tube walks horizon east -> south -> west (tube leads).

Record: last CONFIRMED degrees, minus a 5-deg safety margin, becomes
ra_travel_deg / dec_travel_deg in ~/.config/sateq/mounts/sirius.conf
(run a fresh `eqtool query`... no: just restart the daemon; counters and
state are untouched by this probe).
"""
import argparse
import json
import math
import subprocess
import sys
import time
from pathlib import Path

DEG_PER_STEP = 360.0 / 9024000.0
SIDEREAL_ASEC_S = 15.0418
AXIS_CHAR = {"ra": "1", "dec": "2"}


def die(msg):
    print(f"probe: {msg}", file=sys.stderr)
    sys.exit(1)


def state_path():
    from os import environ
    d = Path(environ.get("SATEQ_STATE_DIR", Path.home() / ".local/state/sateq"))
    d.mkdir(parents=True, exist_ok=True)
    return d / "envelope-probe.json"


def load_state(reset):
    p = state_path()
    if p.exists() and not reset:
        return json.loads(p.read_text())
    return None


def run(cmd):
    r = subprocess.run(cmd, capture_output=True, text=True)
    return r.returncode, r.stdout + r.stderr


def check_port_free():
    if subprocess.run(["pgrep", "-x", "sateqd"], capture_output=True).returncode == 0:
        die("sateqd is running — stop it first (two owners of the serial port "
            "is forbidden); the probe talks raw to the board")
    out = run(["lsusb", "-d", "0403:6001"])[1]
    if "FTDI" not in out:
        die("no FTDI TTL232R on USB bus — is the interface powered?")


def read_counters(port):
    # eqtool poll prints the raw battery; but simplest: reuse eqtool's own
    # reader via `poll`? No — poll streams. One-shot: use python-pyserial.
    import serial

    def cmd(ser, frame):
        ser.reset_input_buffer()
        ser.write(frame + b"\r")
        buf = b""
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.5:
            chunk = ser.read(64)
            if chunk:
                buf += chunk
                if buf.endswith(b"\r"):
                    break
        return buf.decode(errors="replace").strip()

    def revu24(payload):  # nibble-pair swapped 24-bit (board.cpp)
        if len(payload) != 6:
            die(f"bad payload {payload!r}")
        p = payload
        return int(p[4:6] + p[2:4] + p[0:2], 16)

    with serial.Serial(port, 9600, timeout=1) as ser:
        out = {}
        for axis, name in (("1", "ra"), ("2", "dec")):
            for _ in range(3):
                resp = cmd(ser, b":j" + axis.encode())
                if resp.startswith("=") and len(resp) == 7:
                    out[name] = revu24(resp[1:])
                    break
            else:
                die(f"bad :j{axis} reply: {resp!r} — mount on? cable? daemon dead?")
        ser.reset_input_buffer()
        for axis in (b"1", b"2"):  # belt-and-braces instant stop
            ser.write(b":L" + axis + b"\r")
            ser.read(16)
    return out


def wrap24(raw):
    raw &= 0xFFFFFF
    return raw - (1 << 24) if raw & 0x800000 else raw


def degrees(delta_steps):
    return delta_steps * DEG_PER_STEP


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--axis", choices=["ra", "dec"],
                    help="omit with --reset to only re-datum")
    ap.add_argument("--dir", choices=["+", "-"], default="-")
    ap.add_argument("--deg", type=float, default=5.0,
                    help="step size (degrees of axis rotation; negative = back up)")
    ap.add_argument("--x", type=float, default=60.0,
                    help="slew rate in x sidereal while stepping")
    ap.add_argument("--port",
                    default="/dev/serial/by-id/usb-FTDI_TTL232R_FTA67MMH-if00-port0")
    ap.add_argument("--reset", action="store_true",
                    help="re-datum: current counters become 0 deg (mount at home!)")
    ap.add_argument("--force-motion", action="store_true",
                    help="acknowledge that this physically moves the mount")
    args = ap.parse_args()

    if not args.force_motion:
        die("this probe physically moves the mount: re-run with --force-motion")

    check_port_free()
    st = load_state(args.reset)
    if st is None:
        c0 = read_counters(args.port)
        st = {"ra_datum_cnt": c0["ra"], "dec_datum_cnt": c0["dec"],
              "note": "datumed here: physically confirm this is home (az 90 el 0)"}
        state_path().write_text(json.dumps(st, indent=1))
        print(f"[datum] recorded at counters RA=0x{c0['ra']:06X} DE=0x{c0['dec']:06X}")
        if args.reset or args.axis is None:
            print("[reset] datumed; counters now read 0 deg from here")
            return
    if args.axis is None:
        print("[info] no --axis: re-datum only")
        return

    sign = 1.0 if args.dir == "+" else -1.0
    deg = sign * abs(args.deg)
    seconds = max(1.0, round(abs(deg) * 3600.0 / SIDEREAL_ASEC_S / args.x, 1))
    direction = 1 if deg > 0 else -1

    before = read_counters(args.port)
    d_before = degrees(wrap24(before[args.axis] - st[f"{args.axis}_datum_cnt"]))
    print(f"[step] {args.axis} at {d_before:+.2f} deg from datum -> {deg:+.2f}")
    # wind the motor
    eqtool = Path(__file__).resolve().parents[2] / "build" / "eqtool"
    rc, out = run([str(eqtool), "--port", args.port, "rate",
                   "--axis", AXIS_CHAR[args.axis], "--x", str(args.x * direction),
                   "--seconds", str(seconds), "--force-motion"])
    print(out.strip())
    if rc != 0:
        die(f"eqtool rate failed (rc={rc}) — NOT counting this step")

    after = read_counters(args.port)
    line = [f"[pose] counters RA=0x{after['ra']:06X} DE=0x{after['dec']:06X}"]
    for name in ("ra", "dec"):
        d = degrees(wrap24(after[name] - st[f"{name}_datum_cnt"]))
        line.append(f"{name.upper()} {d:+8.2f} deg from datum")
    print("  ".join(line))
    print(f"[step] commanded {args.axis} {deg:+.2f} deg at {args.x:.0f}x for "
          f"{seconds:.1f} s. WATCH THE TUBE: if anything touched or looked "
          f"tight, immediately run: --axis {args.axis} --dir {'-' if sign > 0 else '+'} "
          f"--deg {abs(deg):.1f} --force-motion")
    print(f"[ok] last CONFIRMED envelope test value for {args.axis}: "
          f"{abs(degrees(wrap24(after[args.axis] - st[f'{args.axis}_datum_cnt']))):.1f} deg "
          f"— propose ra/dec_travel_deg = floor(min(confirmed) - 5)")


if __name__ == "__main__":
    main()
