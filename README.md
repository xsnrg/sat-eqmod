# sat-eqmod

Satellite-tracking daemon for SkyWatcher/Orion SynScan EQ-class mounts.

`sateqd` talks the native **motor-board handset-port protocol** (the same
language the hand controller uses: `:cmd<axis>args` + CR at 9600 8N1) and
re-implements the mount's motion ownership directly — no hand controller in
the loop, no INDI, no modified firmware. On the app side it is a plain
**Hamlib `rotctld` server**, so stock satellite trackers (GPredict and any
rotctld client) can steer a Yagi on an EQ mount with zero changes to the app.

The mount is "step-dumb": the board only counts steps. All astronomy
(alt-az ⇄ RA/DEC axis-angle steering, ~10 Hz closed-loop rate control) lives
in the daemon. Tested against a real SynScan Sirius EQ-G (HEQ5-class board)
and characterized against an Orion HDX-110 (EQ8-class board).

## How it works

```
GPredict ──rotctld TCP :4533──▶ sateqd ──FTDI TTL, handset RJ45──▶ motor board
                                   │
                                   ├─ 10 Hz P + velocity-feedforward tracker
                                   ├─ safety: elevation gates, rotation
                                   │   budgets, skip/stall detection,
                                   │   watchdogs, e-stop
                                   └─ session state: persisted counter
                                       reference across restarts
```

- **Write gate / read-only bench mode** (`--read-only`): only pure inquiries
  reach the wire; every write (including stops) is queued and dumped instead.
  A powered mount can be probed with zero chance of motion.
- **Session layers**: station config + per-mount profile + persisted state;
  a live mount-code mismatch with the profile makes the daemon refuse before
  any byte is written.
- **Protocol details the vendor docs gloss over** (`:F` arming before `:J`,
  running-axis NAK rules, board acceleration ramps, deceleration tails) are
  handled from live measurement — see `AGENTS.md` for the engineering journal.

## Build

```sh
cmake -B build && cmake --build build
ctest --test-dir build          # unit + simulator closed-loop suites
```

Linux, C++20, no runtime deps beyond libstdc++ (serial + sockets only).

## Run

```sh
# read-only bench session (safe against a powered mount)
sateqd --mount sirius --orientation altaz --read-only

# tracking session (rotctld on :4533; point GPredict at localhost:4533)
sateqd --mount sirius --orientation altaz --verbose
```

Configs in `examples/` (`station.conf` + one profile per mount, installed
under `~/.config/sateq/`). CLI overrides profile overrides station file.

`tools/eqtool` is the swiss-army probe: `info | query | poll | stop | home |
rate` (motion commands need `--force-motion`; `--hold-writes` dry-runs any
sequence and dumps the exact wire frames).

## Hardware

FTDI TTL cable (5 V for Sirius, **3.3 V for the HDX-110**) wired to the RJ45
handset jack: TX→pin 6, RX→pin 5, GND→pins 1/4. **Pins 7/8 are +12 V — never
to the FTDI.** Pin 5 idles high whenever the mount is on: that UART is the
mount CPU, alive and waiting.

## Safety notes

- The board NAKs motion (`!4`) until each axis is armed with `:F` once per
  power cycle; `:Q1 55AA` drops it into its bootloader — the daemon's write
  whitelist is the only thing between a curious probe and that, so keep it
  that way.
- Grease old mounts before high-rate runs; this project's first gearheads
  were ten-year-old dried grease that passlessly resembles a dead cable.

## License

Apache-2.0 — see [LICENSE](LICENSE).
