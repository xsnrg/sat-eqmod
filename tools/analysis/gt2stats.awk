# gt2stats.awk — per-pass-epoch push-cadence stats for sateqd --record CSVs.
# Epoch = group of consecutive pushes with inter-arrival gap <= 300 s.
# Handles both tap layouts (auto-detects the unix-epoch column; elevation
# is the field immediately before it):
#   3-col: az,el,unix_epoch          4-col: epoch_s,mono_s,az_deg,el_deg
BEGIN { FS = ","; ep = 0; tot = 0 }
{
  tcol = 0
  for (i = 1; i <= NF; i++) if ($i + 0 > 1500000000 && $i + 0 < 2200000000) tcol = i
  if (tcol == 0) next                      # header or malformed line
  if (NF >= 4) el = $4 + 0                 # 4-col tap: epoch_s,mono_s,az_deg,el_deg
  else if (tcol == NF && NF == 3) el = $2 + 0   # 3-col tap: az,el,epoch
  else { bad++; next }
  t = $tcol + 0
  if (tot > 0 && t - prev > 300) ep++
  if (tot > 0) { g[ep, gsz[ep]++] = t - prev }
  cnt[ep]++; if (el > 0) trk[ep]++
  if (cnt[ep] == 1) t0[ep] = t
  t1[ep] = t; prev = t; tot++
}
END {
  printf "%-6s %12s %12s %8s %6s %8s %8s\n", "epoch", "first_ts", "last_ts", "pushes", "el>0", "p50_s", "p95_s"
  for (e = 0; e <= ep; e++) {
    m = gsz[e] + 0
    for (j = 0; j < m; j++) s[j] = g[e, j]
    for (i = 1; i < m; i++) { v = s[i]; j = i - 1; while (j >= 0 && s[j] > v) { s[j+1] = s[j]; j-- }; s[j+1] = v }
    p50 = (m ? s[int((m - 1) * 0.50)] : 0)
    p95 = (m ? s[int((m - 1) * 0.95 + 0.9999)] : 0)
    printf "%-6d %12.0f %12.0f %8d %6d %8.1f %8.1f\n", e, t0[e], t1[e], cnt[e], trk[e] + 0, p50, p95
  }
  printf "epochs=%d total_pushes=%d\n", ep + 1, tot
}
