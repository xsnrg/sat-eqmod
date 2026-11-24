# 10 Hz rotctld p-poll recorder: catches link stalls as latency spikes
import socket, time, sys
out = open("/tmp/opencode/dive-monitor.csv", "w", buffering=1)
s = socket.create_connection(("127.0.0.1", 4533), timeout=2)
t0 = time.monotonic()
out.write("t,ms,az,el\n")
fail = 0
while True:
    try:
        t = time.monotonic()
        s.sendall(b"p\n")
        r = s.recv(64).decode()
        lat = (time.monotonic() - t) * 1000
        lines = r.strip().split("\n")
        if lines[-1].startswith("RPRT") and lines[-1] != "RPRT 0":
            raise ValueError(r.strip())
        out.write("%.2f,%.0f,%s,%s\n" % (t - t0, lat, lines[0], lines[1]))
        fail = 0
    except Exception:
        fail += 1
        if fail == 1:
            print("poll failure", flush=True)
        try:
            s.close()
        except OSError:
            pass
        if fail > 50:
            sys.exit(1)
        time.sleep(0.2)
        s = socket.create_connection(("127.0.0.1", 4533), timeout=2)
    time.sleep(0.1)
