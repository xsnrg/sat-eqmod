# GT2-style dive client: push (0, 0) every 1.5 s, poll p, 100 s session
import socket, time
s = socket.create_connection(("127.0.0.1", 4533), timeout=3)
t0 = time.monotonic()
while time.monotonic() - t0 < 100:
    t = time.monotonic() - t0
    s.sendall(b"P 0 0\n")
    r1 = s.recv(64).decode().strip()
    s.sendall(b"p\n")
    time.sleep(0.15)
    r2 = s.recv(64).decode().strip().replace("\n", " ")
    print("%6.1f push=%s read=%s" % (t, r1, r2), flush=True)
    time.sleep(1.35)
s.sendall(b"q\n")
