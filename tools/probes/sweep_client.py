#!/usr/bin/env python3
"""Scripted live-validation sweep for sateqd (GT2-like client).

Pushes a smooth trajectory at 1 Hz, polls p every 2 s, and exercises the
disconnect/reconnect (watchdog-hold) path mid-sweep. Logs every push+poll to
CSV for offline smoothness/error analysis.
"""
import socket, time, sys, csv

HOST, PORT = "127.0.0.1", 4533
OUT = sys.argv[1] if len(sys.argv) > 1 else "/tmp/sweep-log.csv"

def connect():
    s = socket.create_connection((HOST, PORT), timeout=10)
    return s.makefile("rw")

def sendline(f, cmd):
    f.write(cmd + "\n"); f.flush()

def readline(f):
    line = f.readline()
    if not line:
        raise ConnectionError("closed")
    return line.strip()

def poll(f):
    sendline(f, "p")
    az = float(readline(f)); el = float(readline(f))
    return az, el

def interp(t0, t, spec):
    """spec: list of (t_rel_s, az, el) waypoints; piecewise linear, hold after."""
    if t <= spec[0][0]:
        return spec[0][1], spec[0][2]
    for (ta, a1, e1), (tb, a2, e2) in zip(spec, spec[1:]):
        if t <= tb:
            f = (t - ta) / (tb - ta)
            return a1 + f * (a2 - a1), e1 + f * (e2 - e1)
    return spec[-1][1], spec[-1][2]

def main():
    spec = [
        (0.0,   90.0,  0.0),   # start = park pose
        (60.0, 105.0, 15.0),   # phase A: 0.25 deg/s az + el climb
        (90.0, 105.0, 15.0),   # phase B: settled hold 30 s
        (135.0, 105.0, 15.0),  # phase C: 45 s disconnect window (hold pose)
        (195.0, 90.0,  5.0),   # phase D: return sweep at same rate
        (225.0, 90.0,  5.0),   # final hold 30 s
    ]
    disc_start, disc_end = 90.0, 135.0
    t0 = time.monotonic()
    s = connect()

    log = open(OUT, "w", newline="", buffering=1)  # line-buffered: live tailing
    w = csv.writer(log)
    w.writerow(["t", "phase", "push_az", "push_el", "read_az", "read_el"])
    next_poll = 0.0
    while True:
        t = time.monotonic() - t0
        if t > spec[-1][0] + 5:
            break
        if disc_start <= t < disc_end:
            if s is not None:
                s.close(); s = None
                print(f"t={t:6.1f}  DISCONNECTED (watchdog-hold test)", flush=True)
            time.sleep(1.0)
            continue
        if s is None:
            s = connect()
            print(f"t={t:6.1f}  RECONNECTED", flush=True)
        az, el = interp(t0, t, spec)
        phase = "A" if t < 60 else "B" if t < 90 else "D" if t < 195 else "E"
        sendline(s, f"P {az:.3f} {el:.3f}")
        if "RPRT" not in (r := readline(s)):
            print(f"bad reply to P: {r}"); sys.exit(1)
        if t >= next_poll:
            raz, rel = poll(s)
            next_poll = t + 2.0
            w.writerow([f"{t:.1f}", phase, f"{az:.3f}", f"{el:.3f}", f"{raz:.4f}", f"{rel:.4f}"])
            err = ((raz - az + 180) % 360) - 180, rel - el
            print(f"t={t:6.1f} {phase} push=({az:7.2f},{el:5.2f}) read=({raz:7.2f},{rel:5.2f}) err=({err[0]:+6.2f},{err[1]:+5.2f})", flush=True)
            log.flush()
        time.sleep(1.0)
    if s: s.close()
    log.close()
    print("sweep complete")

main()
