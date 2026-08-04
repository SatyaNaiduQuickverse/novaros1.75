# Hardware bring-up

Wire the vehicle, then bring up and verify every I/O path with the bench
harness before adding the operational module. The whole sequence below runs
against the idle placeholder — nothing else is needed to prove the plumbing.

**Props off for all of it.** A pilot with a transmitter must be able to take
control instantly at every moment.

## Hardware this is written against

| | |
|---|---|
| Flight controller | STM32F405 (board changed 2026-08-04; previously SpeedyBee F7 V3) |
| Firmware | Betaflight 4.5.1, MSP API 1.46, **with the MSP Override option** |
| Companion | Raspberry Pi 5, FC on USB-C (VCP) |
| Receiver | ELRS/CRSF on UART2, pilot transmitter always on |

`config/vehicle.yaml` pins the full `/dev/serial/by-id/` path. This matters
concretely: an ESP32 on this Pi enumerates as USB CDC and currently holds
`ttyACM0`, leaving the FC on `ttyACM1`. Raw `ttyACM` numbering would talk to the
wrong device.

**A board swap invalidates more than the port.** Re-measure all of these before
trusting a new FC. `tools/find_fc.py` identifies it and prints the config block;
`bringup preflight` reads the aux bindings straight from the firmware and
cross-checks them against the config; `bringup calib-imu` re-measures the IMU.

| Property | Why it does not carry over |
|---|---|
| by-id path | different target / serial |
| MSP transaction ceiling | firmware+target dependent — 99/s on F722, **62/s on F405** |
| `acc_per_g` | depends on that board's `acc_1G` and its MSP scaling |
| IMU axis map, attitude signs | depends on how the board is mounted |
| aux switch map, `override_mask` | whatever is flashed on that board |
| box bit positions | resolved at runtime from `MSP_BOXIDS` — this one is safe |

## FC configuration (saved to EEPROM)

```
set msp_override_channels_mask = 15     # binary 1111 = channels 1-4 only
aux <slot> 50 3 1700 2100 0 0           # box id 50 = MSP OVERRIDE, on ch8
```

Verified present on the current board 2026-08-04 by reading `MSP_MODE_RANGES`:
ANGLE on AUX1/ch5 (900-1300), HEADFREE AUX3/ch7, MSP OVERRIDE AUX4/ch8
(1700-2100), ARM AUX5/ch9 (1700-2100). `bringup preflight` prints this and
fails if it disagrees with `config/vehicle.yaml`.

Switch map for this airframe and pilot transmitter:

| Channel | Function |
|---|---|
| ch1-4 | roll / pitch / yaw / throttle, 1000-2000 us (stick positions, not RPM) |
| ch5 | ANGLE mode — active when the switch is **LOW** (900-1200) |
| ch8 (AUX4) | MSP OVERRIDE — companion takeover at >= 1700 |
| ch9 (AUX5) | ARM — **the companion never touches or spoofs this** |

CLI gotchas that have cost time here: `save`/`exit` reboots the FC and drops USB
mid-read, so expect re-enumeration and reconnect by the by-id path; a crashed
script can leave the FC stuck in CLI mode, where MSP times out until you send
`exit\n`; a 4.3-era `diff all` contains `batch start` with no `batch end`, so
`save` silently no-ops until you send `batch end`; and EEPROM writes can take
over a second, so use a generous timeout.

## Bring-up order

Get each step green before starting the next.

### Step 0 — prove the software runs with no hardware

```bash
make test          # 94 offline tests
make loop-fake     # full loop structure + logging, fake IMU, fake vision, no FC
python3 tools/log_analyze.py ~/logs/run_*.jsonl
```

### Step 1 — the link (FC powered, props off)

```bash
make preflight     # firmware, override support, switch positions, RC link
make link          # 5 s of live attitude + IMU
```

`preflight` fails immediately if the firmware lacks MSP Override. If the RC
link is down, the usual cause is the ELRS receiver sitting in WiFi mode, where
it outputs no CRSF at all: power-cycle the battery *and* USB with the
transmitter already on.

### Step 2 — IMU scale and axis map

```bash
make calib         # you tilt the airframe; it prints YAML to paste back
```

Betaflight's `MSP_RAW_IMU` units and sensor-frame orientation vary by build and
by board, and a wrong axis map is invisible downstream — it just points every
bearing slightly the wrong way. So this step measures rather than assumes:
level for the accelerometer scale and the gravity axis, nose-up and roll-right
for the two horizontal axes and the attitude signs. Paste the printed block
into `config/vehicle.yaml` (it sets `verified: true`), then:

```bash
make imu           # level -> acc ~[0, 0, -9.81] FRD; q tracks hand-tilts
```

Body frame is FRD throughout: x forward, y right, z down. World is NED.

### Step 3 — the RC command path (props off)

```bash
make rc            # streams idle RC for 5 s
```

Confirm the Betaflight receiver tab shows the channels, and confirm physically
that the pilot's transmitter can take control at any moment.

```bash
make override      # guided takeover, handback, and dead-man test
```

This one proves the three things that matter before any powered work: the
companion can take the sticks, the pilot can take them back, and stopping the
stream *alone* hands control back with the switch still up.

### Step 4 — the vision adapter

Wire the tracker source and set the calibration, then:

```bash
make vision
python3 -m tools.bringup vision --tracker-url http://localhost:8080/telemetry
```

Point the camera at a known object; the bearing must point the right way as
both the object and the vehicle move.

### Step 5 — the full plumbing loop (props off)

```bash
make loop
python3 tools/log_analyze.py ~/logs/run_*.jsonl
```

Expect good IMU state, a bearing whenever the object is in view, idle sticks
streaming, a high measured-fraction, and no abort. That is "all plumbing
verified".

## Calibrations

Each of these silently biases the bearing rather than failing loudly, which is
why they get their own step rather than being assumed.

- **MSP_RAW_IMU scaling and axis map** — `make calib`.
- **Camera intrinsics and lens distortion** — OpenCV checkerboard, into
  `camera.fx/fy/cx/cy/dist_coeffs`.
- **Camera-to-body mounting (boresight)** — the `cam_fwd/right/up` basis, from
  how the camera actually sits in the airframe.
- **IMU-to-body mounting** — folded into the IMU driver's output.
- **Camera-frame to IMU-sample time alignment** — a mismatch looks exactly like
  latency, and shows up as a bearing that lags during rotation.

## Bench-test note

With the craft strapped down and ANGLE mode active, the self-levelling PID
integrates an error it can never correct. Motor readbacks creep upward at
constant command and one motor may pin at the mixer floor. This is normal on a
stand and disappears in free flight — but don't sit armed on the bench for long
stretches.

## Last step — the operational module

When everything above is green, replace `companion/command_module.py`. It is
constructed the same way (`fc, imu, vision`) and called the same way each tick
(`.step(t)`). Nothing else in the plumbing changes. Its contract:

- `step(t)` must not block — it runs inside the control loop at `fc.rc_hz`.
- Never write aux channels.
- Clamping is not its job; `safety.aetr_frame()` enforces the envelope on every
  frame regardless of what is asked for.

Raise `limits.profile` from `bench` to `tethered` to `flight` one step at a
time, and only when the step below it is green.

## The dedicated companion IMU (ESP32-C6 + MPU6500)

The flight-article attitude source. `FCIMU` is bring-up only: it costs one MSP
transaction per sample from a budget that already caps the control rate, and
the FC's attitude is quantised to 0.1 deg.

```
Pi  <--USB CDC 200Hz--  ESP32-C6  <--I2C 400kHz--  MPU6500 @0x68
                        SCL GPIO19 / SDA GPIO20
```

The ESP32 ships **raw counts only** (18-byte frames, `esp32/main.py`). Scaling,
bias removal, the axis map and the Mahony filter all run on the Pi
(`companion/imu_esp32.py`), so they can be changed without reflashing — and
MicroPython on the C6 cannot hold a 200 Hz filter anyway.

### Flashing from scratch

```bash
python3 -m venv --system-site-packages .venv-esp
.venv-esp/bin/pip install esptool mpremote
curl -LO https://micropython.org/resources/firmware/ESP32_GENERIC_C6-<ver>.bin
.venv-esp/bin/esptool --chip esp32c6 --port <esp-by-id> write-flash 0x0 ESP32_GENERIC_C6-<ver>.bin
.venv-esp/bin/mpremote connect <esp-by-id> cp esp32/main.py :main.py
```

MicroPython is used deliberately: no ESP-IDF toolchain, and the bridge becomes
a Python file that iterates in seconds. C6 native USB is GPIO12/13, so GPIO19/20
are free — note this would **not** hold on an ESP32-S3, where 19/20 are the USB
D-/D+ pins.

### Calibrating

In this order — each step's result feeds the next:

```bash
python3 -m tools.bringup imu32 --calibrate   # gyro bias, at rest
python3 -m tools.bringup imu32 --accel-cal   # per-axis offset/scale, by tumbling
python3 -m tools.bringup imu32 --axis-map    # sensor->body FRD, needs tilts
python3 -m tools.bringup imu32               # live state + link health
```

**The gyro bias is not optional on this part.** Measured ~9 deg/s on one axis;
uncorrected it fights the gravity correction and leaves a standing 3.4 deg
attitude error. With it applied the estimate converges to ~0.35 deg.

⚠️ **The accel scale is not a single number on this part, and the older claim
that "magnitude alone gives counts-per-g in any orientation" is FALSE here.**
Measured `|a|` = 2400 counts with gravity on sensor z and 1997 on sensor x,
both stationary — a fixed per-axis property, reproduced an hour apart. It is a
per-axis zero-g offset of ~0.17 g, and because the Mahony filter normalises the
accel vector, that offset becomes ~10 deg of standing attitude error while a
pure scale error would have cancelled harmlessly. Use `--accel-cal`, and run it
**before** `--axis-map`, which picks signs from the largest raw component and
can be mis-called by that offset. Full write-up and the ellipsoid maths:
[ESP32_BRIDGE_FAULTS.md](ESP32_BRIDGE_FAULTS.md).

The **axis map** needs level plus tilts, and ships unverified until measured.

### Known limits

- **No magnetometer.** The board may be labelled MPU9250, but `WHO_AM_I` reads
  `0x70` (MPU6500) and no AK8963 appears at `0x0C` even with I2C bypass enabled.
  Roll and pitch are gravity-observable; **yaw has no absolute reference and
  drifts at roughly 0.3 deg/s**. Anything needing true heading must take it from
  the FC's compass, or initialise yaw from the FC once at engage.
- An erased ESP32 boot-loops (`invalid header: 0xffffffff`, `TG0_WDT_HPSYS`),
  re-enumerating every ~2.3 s. That also churns `ttyACM` numbering, which is
  another reason the FC is pinned by by-id path.
- 🚨 **The bridge can be enumerated and totally silent, indefinitely** — once
  measured at 19 minutes, cured only by a chip reset. On the C6 the
  USB-Serial-JTAG is a *hardware* peripheral, so appearing under
  `/dev/serial/by-id/` proves the chip has power and nothing more. The reader
  now opens the port *before* rebooting the board into it, and auto-recovers
  from silence. Never debug this from the device list alone:
  [ESP32_BRIDGE_FAULTS.md](ESP32_BRIDGE_FAULTS.md).
- **The running firmware cannot be interrupted while streaming.** Every
  host→device write times out, so Ctrl-C never lands and `mpremote` cannot
  connect. Reflashing needs the ROM download mode (esptool) or a full erase.
  Any handle to this device must set `write_timeout`, or writes block forever.
