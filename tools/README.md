# Tool directories

- `sateqd`, `eqtool`, `replay_gt2` — built from src/ (see README).
- `probes/` — raw wire probes that talk the SkyWatcher protocol
  DIRECTLY on the serial port (never while sateqd is live).
  `raw_j_battery.py` is the reference pattern (swap-aware `revu24()`).
- `live/` — rotctld clients run against a live sateqd session:
  pass monitors (`passmon2.py`), motion probes (`verify_rise.py`,
  `settle_check.py`), dive stressors, the camera oracle helpers
  (`snap.py`, `cam_freeze.py`). These HOLD their socket — see the
  GT2 stop-semantics note in AGENTS.md before scripting one-shots.
- `analysis/` — offline number crunchers over replay traces and logs
  (`errbucket.awk`, `gt2stats.awk`, `errstats.py`, `wind.cpp`).

Live bench scripts used to live in /tmp on bench and dev machines; that
is ephemeral (boot-wiped). Put changes here, deploy via git bundle.
