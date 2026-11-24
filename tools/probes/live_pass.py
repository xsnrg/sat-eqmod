#!/usr/bin/env python3
"""Live Sirius session: small altaz acquisition sweep + track-gate band test.
Consented motion, small travel, always leaves with stop."""
import socket, time, sys

HOST, PORT = "127.0.0.1", 4533

def main():
    s = socket.create_connection((HOST, PORT), timeout=5)
    s.settimeout(5)

    def send(cmd):
        s.sendall((cmd + "\n").encode())
        return s.recv(256).decode().strip()

    def pos():
        s.sendall(b"p\n")
        time.sleep(0.15)
        d = s.recv(256).decode().split()
        return float(d[0]), float(d[1])

    def target(az, el):
        r = send(f"P {az:.3f} {el:.3f}")
        assert r.startswith("RPRT 0"), r

    az, el = pos()
    print(f"start pose: az={az:.2f} el={el:.2f} (expect ~90/0 level EAST)")

    print("phase A: sweep az 92->96, el 1->6 (24 s, 1 Hz updates)")
    err_max = 0.0
    t0 = time.time()
    while True:
        t = time.time() - t0
        if t > 24:
            break
        ta, te = 92.0 + t * 0.17, 1.0 + t * 0.21
        target(ta, te)
        time.sleep(0.95)
        a, e = pos()
        err = ((a - ta) ** 2 + (e - te) ** 2) ** 0.5
        if t > 6:
            err_max = max(err_max, err)
    print(f"phase A done: steady-state max az/el error {err_max:.3f} deg")

    print("phase B: target az 96 el -3 (in band, below optical horizon)")
    target(96.0, -3.0)
    time.sleep(8)
    a, e = pos()
    print(f"  pose now az={a:.2f} el={e:.2f} (moving toward -3 el = tracking below horizon OK)")

    print("phase C: target el -6 (below stop gate: must HOLD, not chase)")
    target(96.0, -6.0)
    time.sleep(2.0)
    a1, e1 = pos()
    time.sleep(4.0)
    a2, e2 = pos()
    moved = ((a2 - a1) ** 2 + (e2 - e1) ** 2) ** 0.5
    print(f"  pose az={a2:.2f} el={e2:.2f}; motion while below gate: {moved:.3f} deg "
          f"({'HALTED OK' if moved < 0.15 else 'STILL MOVING - BUG'})")
    assert moved < 0.15

    print("phase D: target back to el +4: auto re-arm, rise")
    target(96.0, 4.0)
    time.sleep(8)
    a, e = pos()
    print(f"  pose now az={a:.2f} el={e:.2f} (el rising toward +4 = re-armed)")

    print("phase E: return to park pose az 90 el 0")
    target(90.0, 0.0)
    time.sleep(15)
    a, e = pos()
    print(f"  pose now az={a:.2f} el={e:.2f}")
    print(send("S"))
    print(send("\\stop"))
    print(send("q"))
    print("LIVE PASS DONE")

if __name__ == "__main__":
    sys.exit(main())
