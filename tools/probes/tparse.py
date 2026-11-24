import re, sys
txt = open(sys.argv[1]).read().splitlines()
out = []
cmd = None; buf = ""
for line in txt:
    m = re.search(r'write\(3, "(:[^"]*)"', line)
    if m:
        cmd = m.group(1).replace("\\r","").replace("\\n","")
        buf = ""
        continue
    m = re.search(r'read\(3, "([^"]*)"', line)
    if m and cmd is not None:
        ch = m.group(1)
        if ch == "\\r":
            out.append((cmd, buf)); cmd = None
        else:
            buf += ch
seen = {}
order = []
for c, r in out:
    key = c
    if key not in seen:
        seen[key] = r; order.append(key)
for c in order:
    print("{:12s} -> {}".format(c, seen[c]))
print("--- total exchanges:", len(out))
