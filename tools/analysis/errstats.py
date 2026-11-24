import re, math, sys, collections
pat = re.compile(r"^t=\s*([\d.]+) tgt=\(\s*([-\d.]+),\s*([-\d.]+)\) cur=\(\s*([-\d.]+),\s*([-\d.]+)\)")
buckets = collections.defaultdict(list)
for line in open(sys.argv[1]):
    m = pat.match(line)
    if not m: continue
    t, az, el, caz, cazel = map(float, m.groups())
    if el <= 0.05: continue
    daz = az - caz
    daz = (daz + 180) % 360 - 180
    e = math.hypot(daz, el - cazel)
    buckets[int(t // 120) * 120].append(e)
for k in sorted(buckets):
    v = buckets[k]
    rms = math.sqrt(sum(x*x for x in v)/len(v))
    over = sum(1 for x in v if x > 1.0)
    print(f"{k:5d}  n={len(v):5d}  rms={rms:6.3f}  max={max(v):7.3f}  >1deg:{over}")
