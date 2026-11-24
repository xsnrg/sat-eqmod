import socket, time
s = socket.create_connection(("127.0.0.1", 4533), timeout=5)
f = s.makefile("rw", buffering=1)
def cmd(c):
    f.write(c + "\n"); return f.readline().strip()
def pos():
    f.write("p\n")
    return float(f.readline()), float(f.readline())
az, el = pos()
print(f"start: az={az:.2f} el={el:.2f}")
t0 = time.time(); last = -99
while time.time() - t0 < 30:
    cmd("P 90 10")          # hold target like GT2 follow gate
    time.sleep(0.9)
    az, el = pos()
    if abs(el - last) > 0.2 or time.time() - t0 > 29:
        print(f"t={time.time()-t0:5.1f}s  az={az:7.2f}  el={el:6.2f}"); last = el
time.sleep(2)
az, el = pos()
print(f"settled: az={az:.2f} el={el:.2f}")
cmd("q")
