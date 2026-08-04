# Working on this repo

Companion connector layer for a counter-UAS interceptor: Raspberry Pi 5 →
Betaflight FC over MSP override, with a dedicated ESP32 IMU. Real hardware,
real motors, a pilot holding a transmitter. Start with `README.md` for what
the code is; this file is for what will bite you.

## Ground rules

- **The pilot owns ARM and the override switch.** `FCLink.arm()` raises on
  purpose. Never try to arm, and never spoof ch8/ch9 — they are read-only.
  Do not run anything that spins a motor without an explicit, current
  go-ahead; "props are off" from an hour ago is not one.
- **`companion/safety.py` is the only place channel bytes are packed** and the
  only place clamps apply. Everything reaching the FC goes through
  `aetr_frame()`. If you find a second path, that is a bug.
- **AETR asymmetry:** `MSP_SET_RAW_RC` takes raw receiver order — throttle at
  index **2**. `MSP_RC` returns roll, pitch, yaw, throttle — throttle at index
  **3**. Both verified on hardware; they are not the same order.
- **Measure, do not assume.** Nearly every constant in `config/vehicle.yaml`
  started as a plausible default that turned out to be wrong on this airframe.
  A value with no measurement behind it belongs behind `verified: false`.

## Things that are per-board, not constants

- **MSP transaction ceiling.** F722/BTFL 26.6.1 ≈ 99 txn/s; F405/BTFL 4.5.1 ≈
  62. Shared by the RC stream, telemetry (2/cycle) and the watchdog (3/check).
  Oversubscribing never errors — every rate just silently degrades. Check
  `FCLink.txn_budget()`.
- **The MSP override timeout.** The F722 handed control back in ~250 ms. **The
  F405 never does** — it held stale sticks for 5+ s, and the buffer survives a
  companion restart. Aborts therefore *keep streaming idle* rather than
  stopping the stream. See the 🚨 section in `docs/SETUP_HARDWARE.md`.
- **`acc_per_g`.** Depends on the board's `acc_1G` *and* its MSP scaling.

## The ESP32 IMU will fool you

Read **`docs/ESP32_BRIDGE_FAULTS.md`** before debugging or changing
`companion/imu_esp32.py` or `esp32/main.py`. Two measured faults, both of which
look exactly like nothing being wrong:

1. **The bridge can be enumerated and permanently silent** (measured: 19
   minutes). On the C6 the USB-Serial-JTAG is a *hardware* peripheral, so
   presence under `/dev/serial/by-id/` proves the chip has power and nothing
   else. The reader opens the port *before* rebooting the board into it, and
   auto-recovers from silence. Any handle to this device needs
   `write_timeout` or writes block forever.
2. **The accelerometer's reported `|a|` depends on which axis gravity lands
   on** — 2400 counts on z, 1997 on x, both stationary, reproduced an hour
   apart. A single `accel_per_g` cannot express that. It is a per-axis offset,
   and since Mahony normalises the accel vector, a scale error cancels while an
   offset becomes ~10 deg of standing attitude error.

Calibrate in this order — each result feeds the next:
`--calibrate` → `--accel-cal` → `--axis-map`.

## Verifying claims

- **Do not trust a status report over the tree.** A handoff message once
  claimed three safety features were done; `grep` showed none of them existed.
  Check the code.
- **Gate hardware tests on the state they assume.** An abort test once read
  `[1500, 1503, 1497, 989]` and looked green — that was the *pilot's* sticks,
  because the override switch was down. It proved nothing. Gate on
  `override_active()`.
- **Prove liveness before blaming the operator.** "Nobody moved it" and "the
  reader is frozen" produce identical logs. A live MPU6500 at rest shows 2–4
  counts of standard deviation and changes on nearly every sample.
- **A calibration that cannot be solved must refuse, not guess.** An ellipsoid
  fits through any patch of its own surface and will return a confident, wrong
  centre. These calibrations decide which way is down.

## Commands

```bash
make test                                    # offline, no hardware
python3 -m tools.bringup preflight           # who is on the port, what is unverified
python3 -m tools.bringup imu32               # live ESP32 attitude + link health
python3 tools/find_fc.py                     # identify the board and measure its MSP ceiling
```

`config/vehicle.yaml` carries the measurement and the date behind each value.
Keep that habit — the comments are the audit trail.
