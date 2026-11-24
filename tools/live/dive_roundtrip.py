# repeated dive stress: 0<->90 el 0, hold between, with stall instrumentation
import socket, time
def connect():
    return socket.create_connection(("127.0.0.1", 4533), timeout=3)
s = connect()
t0 = time.monotonic()
def push_to(az, hold):
    global s
    t_end = time.monotonic() + hold
    while time.monotonic() < t_end:
        try:
            t = time.monotonic() - t0
            s.sendall(("P %g 0\n" % az).encode())
            r1 = s.recv(64).decode().strip()
            s.sendall(b"p\n")
            time.sleep(0.15)
            r2 = s.recv(64).decode().strip().replace("\n", " ")
            print("%6.1f az%d push=%s read=%s" % (t, az, r1, r2), flush=True)
        except Exception as e:
            print("%6.1f az%d STALL %s" % (time.monotonic() - t0, az, e), flush=True)
            try:
                s.close()
            except OSError:
                pass
            time.sleep(0.5)
            s = connect()
        time.sleep(1.2)
for az, hold in [(90, 45), (0, 30), (90, 45), (0, 30)]:
    push_to(az, hold)
try:
    s.sendall(b"q\n")
except OSError:
    pass
print("roundtrip complete", flush=True)
