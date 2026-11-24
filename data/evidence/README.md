# Bench evidence archive

Raw output of live-bench forensics that AGENTS.md sections cite by
narrative. Placed here 2026-11-19 cleanup because the bench machine's /tmp was
cleared on boot.

**Git policy (user decision 2026-11-19): `.gz` files are NOT committed.**
This directory on dev-machine disk is the permanent archive for the gzipped
daemon/client logs; keep new ones here but do not `git add -f` them.
Committed content: CSVs, `rawj*.out` raw-probe batteries, and this README.

- `rawj*.out` — the 2026-11-17 `:j`-battery raw probe outputs proving
  both axes run exactly commanded rates after `:F` arming + swapped
  `:I` periods (the "corrupt readback" false-alarm evidence).
- `sateqd-sweep{2..7}.log.gz`, `sweep{2..7}-client.log.gz`,
  `opencode-live-sweep*.csv` — live sweep campaign 2026-11-17/18
  (:K-stall cascade root-cause through sweep-7 PASS).
- `sateqd-1118am.log.gz` — first `--mount auto` + state-resume session.
- `sateqd-1118pm6.log.gz` — AO-73 live-pass forensics log (backlash
  limit-cycle + GT2 double-push `:K` storm; push CSV lives in
  `../gt2-iqwork-2026-11-18/`).
- `sateqd-dive{1,2}.log.gz`, `dive-monitor-run{1,2,3}.csv` — dive
  stress / link-latency forensics 2026-11-18 (DEC `:j` freeze).
- `backlash-live.log.gz` — live backlash measurement run output
  (numbers also in `~/.local/state/sateq/sirius.cal` and AGENTS).
- `fo29-rcs6.txt.gz` — residual `:K` class census at shipped
  iteration-2 defaults (HIGH-scale ~46, band-edge ~25, flips ~4).
- `opencode-live-1117.csv`, `sweep-log` era captures — early sweeps.
