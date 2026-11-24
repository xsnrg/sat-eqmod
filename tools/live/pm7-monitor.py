import socket, time, sys, csv, datetime, os
HOST, PORT = "127.0.0.1", 4533
OUT = os.path.expanduser("~/data/pm7-monitor-latency.csv")
s = socket.create_connection((HOST, PORT), timeout=5)
s.settimeout(5)
f = open(OUT, "w", newline="")
w = csv.writer(f); w.writerow(["mono","utc","lat_ms","az","el"])
buf = ""
def readline_timeout(t):
    global buf
    end = time.monotonic() + t
    while time.monotonic() < end:
        nl = buf.find("\n")
        if nl >= 0:
            line, buf = buf[:nl], buf[nl+1:]
            return line
        s.settimeout(max(0.05, end - time.monotonic()))
        try:
            d = s.recv(4096)
        except socket.timeout:
            return None
        if not d: return None
        buf += d.decode(errors="replace")
print("monitor attached", flush=True)
while True:
    t0 = time.monotonic()
    try:
        s.sendall(b"p\n")
        az = readline_timeout(5); el = readline_timeout(5)
        lat = (time.monotonic() - t0) * 1000.0
        w.writerow([f"{t0:.3f}", datetime.datetime.utcnow().isoformat(),
                    f"{lat:.1f}", az, el])
        f.flush()
        if lat > 500: print(f"SLOW {lat:.0f} ms az={az} el={el}", flush=True)
    except Exception as e:
        print("monitor died:", e, flush=True); break
    time.sleep(1.0)
