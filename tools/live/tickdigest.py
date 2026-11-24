import re, sys
pat = re.compile(r"T(\d\d:\d\d:\d\d)\S+ \[TICK\] seq=(\d+).*hs=(\S+) tg=(\S+).*des=(\S+) err=(\S+) flip=(\d) x=(\S+) sent=(\S+) band=(\S+).*flags=(\S+)")
rows = []
for line in open(sys.argv[1]):
    if "[TICK]" not in line: continue
    m = pat.search(line)
    if not m or m.group(4) == "none": continue
    rows.append(m.groups())
for g in rows[::max(1, len(rows)//45)]:
    print("t=%s seq=%s hs=%s tg=%s des=%s err=%s flip=%s x=%s sent=%s band=%s fl=%s" % g)
print("total target ticks:", len(rows))
