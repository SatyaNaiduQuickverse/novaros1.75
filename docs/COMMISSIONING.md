# Commissioning a unit

Ordered build procedure for one airframe, start to finish. Follow it top to
bottom — each phase depends on the one above, and the gates exist because
skipping them produces a unit that passes every check and flies wrong.

Budget roughly **40 minutes** for a unit once you have done one, of which about
8 are hands-on calibration.

> **Props off for everything here.** The only phase that spins a motor is 6,
> and it says so.

---

## What transfers between units, and what does not

This is the part that decides how much work each unit is. Getting it wrong in
the optimistic direction produces an aircraft calibrated for a different one.

| | transfers | why |
|---|---|---|
| All code, tools, tests | ✅ | one git clone |
| `command_module.so` + its digest pin | ✅ | same binary, same behaviour |
| FC config (`config/fc_diff_all.txt`) | ✅ | aux bindings, mask, PIDs, rates |
| `esp32/main.py` | ✅ | same firmware |
| Attitude sign conventions (`imu.pitch_sign` etc.) | ✅ * | same FC firmware + board type |
| Stick directions (`roll_right_us`, `nose_down_us`) | ✅ * | same transmitter model + channel map |
| **MSP transaction ceiling** | ❌ | per board. F722 ≈ 99/s, F405 ≈ 62/s |
| **`imu32.gyro_bias`** | ❌ | per die, and large (~9 dps) |
| **`imu32.accel_offset` / `accel_per_g_axis`** | ❌ | per die. One read 2400 counts/g on z, 2001 on x |
| **`imu32.mount_matrix`** | ❌ | per physical mounting. This one is 7.5° off square |
| **Camera intrinsics / boresight** | ❌ | per lens and per mount |
| **Motor order → physical corner** | ❌ | per wiring |

\* transfers only if the unit is genuinely identical. Verify rather than
assume — phase 5's six-pose check costs a minute and is the whole point.

**Consequence for N units:** each needs its own calibration block. Keep one
config per unit rather than editing a shared one — see *Per-unit configs* at
the bottom.

---

## Phase 0 — before you start

- [ ] FC flashed with a build that has **MSP Override** (box id 50). Stock
      SPEEDYBEE F7 V3 builds do not; `FCLink.connect()` refuses without it.
- [ ] Receiver bound, transmitter on, model selected
- [ ] ESP32-C6 wired: **SCL GPIO19, SDA GPIO20** to the MPU6500
- [ ] ESP32 mounted **rigidly** to the airframe. It does not have to be square
      — phase 4 measures the angle — but it must not be able to shift, or the
      axis map means nothing
- [ ] Pi 5 with Python 3.11+, both boards on USB
- [ ] **Props off**

---

## Phase 1 — flight controller

```bash
git clone <repo> ~/novaros1.75 && cd ~/novaros1.75
pip install -r requirements.txt
python3 tools/find_fc.py
```

- [ ] Board identified, MSP OVERRIDE present
- [ ] **Record the measured MSP ceiling** — it is per-board and every rate below
      depends on it

Restore the FC configuration: open Betaflight Configurator → CLI, paste
`config/fc_diff_all.txt`, then `save`.

- [ ] Aux bindings present: ARM ch9, ANGLE ch5, HEADFREE ch7, MSP OVERRIDE ch8
- [ ] `msp_override_channels_mask` as intended (11 = pilot keeps throttle)
- [ ] **Accelerometer calibrated in Configurator** on a level surface. Skipping
      this poisons phase 4, because the FC is the reference frame it fits to.

Set the rates in `config/vehicle.yaml` to fit the measured ceiling:
`rc_hz + 2×telemetry_hz + 3×watchdog_hz ≤ 0.9 × ceiling`. Oversubscribing never
errors; every rate silently degrades.

```bash
python3 -m tools.bringup preflight
```

- [ ] Firmware, switches and mask all reported as expected

---

## Phase 2 — ESP32 IMU bridge

```bash
python3 -m venv --system-site-packages .venv-esp
.venv-esp/bin/pip install esptool mpremote
curl -LO https://micropython.org/resources/firmware/ESP32_GENERIC_C6-<ver>.bin
.venv-esp/bin/esptool --chip esp32c6 --port <esp-by-id> write-flash 0x0 ESP32_GENERIC_C6-<ver>.bin
.venv-esp/bin/mpremote connect <esp-by-id> cp esp32/main.py :main.py
```

- [ ] `WHO_AM_I` is 0x70 or 0x71 (the bridge raises otherwise)
- [ ] Streaming: `python3 -m tools.bringup imu32` shows 200 Hz, 0 drops

> Presence under `/dev/serial/by-id/` proves the chip has power and nothing
> more — on the C6 the USB peripheral is hardware. If it enumerates and streams
> nothing, that is the documented wedge; the reader recovers automatically.
> See `docs/ESP32_BRIDGE_FAULTS.md`.

---

## Phase 3 — calibrate the IMU  *(per unit, none of this transfers)*

```bash
make calib-ui        # then open http://<pi>:8720/
```

Do them **in this order** — each feeds the next.

- [ ] **Gyro bias** — `--calibrate`, airframe still. Large on this part and not
      optional; uncorrected it fights the gravity correction
- [ ] **Step 1, accel offset & scale** — 90 s tumble, all six tiles green.
      A single counts-per-g cannot describe these dies
- [ ] **Step 2, axis map vs the FC** — 60 s tumble.
      Gate: **residual < 8°, confidence > 0.8**. Writes `mount_matrix`, which
      keeps the few degrees the snapped `axis_map` would round away

If step 2 fails with a large residual, do **not** re-tumble. Run
`python3 tools/check_fc_attitude.py` first — a wrong euler convention looks
exactly like a badly mounted ESP32, and no amount of re-running detects it.

---

## Phase 4 — verify the signs  *(the gate that matters most)*

Still in the calibration UI.

- [ ] **Six-pose sign check** — hold each position, press its button. **All six
      green on BOTH sensors.**
- [ ] **3a, tracking** — 30 s tilting through both axes, past 40°.
      Gate: worst disagreement **< 8°**
- [ ] **3b, correction opposes the tilt** — run the module against the live IMU
      (output goes to a stand-in; nothing reaches the FC). Tilt past 20° in
      **all four directions** and check each verdict

> Why the six-pose check and not just the cross-check: the axis map fits the
> ESP32 **to the FC's frame**, so a wrong sign in that frame is absorbed rather
> than caught. Both sensors then agree beautifully while both are wrong. Only
> an external reference — your hands, naming the pose — can see it.

Ignore the roll figures in the two nose-vertical poses; they disagree by 10-15°
while pitch matches exactly. That is gimbal lock, not a fault.

---

## Phase 5 — stick directions  *(needed by the self-level check)*

```bash
python3 tools/check_stick_direction.py     # override switch DOWN
```

- [ ] Record `nose_down_us` and `roll_right_us` into `config/vehicle.yaml`

Do not infer one axis from the other. On this airframe both sticks read 2012 at
full deflection, yet correcting nose-up commands pitch **above** 1500 while
correcting right-side-down commands roll **below** it.

---

## Phase 6 — armed bench test  *(motors turn)*

**Props off. Confirm it out loud.** Override switch (ch8) up, ANGLE off (ch5
out of its low band), HEADFREE off.

```bash
python3 -m tools.bringup motors --seconds 60
```

Run it yourself in a terminal you can see — it prompts you to arm, and a prompt
you cannot see is a prompt that does not exist.

- [ ] ROLL RIGHT → **left** motor pair spins up
- [ ] ROLL LEFT → **right** pair
- [ ] PITCH FWD → **rear** pair (nose down)
- [ ] PITCH BACK → **front** pair (nose up)
- [ ] YAW RIGHT / LEFT → opposite diagonals

Expect a clean ~1420 against ~1042 split. Each phase eases off once measured —
on a restrained airframe in ACRO the sticks command a *rate* it cannot deliver,
so a held stick always saturates eventually. That is physics, not a fault.

- [ ] **Motor order → physical corner**, in Configurator's Motors tab. Spin one
      motor at a time and watch which corner turns. MSP readback cannot see
      this: everything above happens *inside* the FC, so a mis-wired ESC looks
      perfect from here

---

## Phase 7 — bind the unit to its hardware

```bash
python3 tools/bind_unit.py --id NOVA-002
```

- [ ] Records the FC's `mcu_id` and the ESP32's MAC into this unit's config
- [ ] Commit it

**Do not skip this.** Everything above measured values that belong to *these
two boards*. Without the binding, deploying this config to another airframe
gives it this one's gyro bias, accel offsets and mount matrix — and nothing
downstream can tell. The numbers stay plausible, `make test` passes, preflight
is green, and the aircraft flies wrong.

With it, `FCLink.connect()` reads the FC's unique id and **refuses** on a
mismatch:

```
WRONG AIRFRAME: config/units/NOVA-002.yaml was measured on FC 0022004c...,
but this board is 0041006b.... Its calibration — gyro bias, accel offsets,
mount matrix — belongs to a different unit.
```

The ESP32 is checked the same way from the MAC in its by-id path. Check any
unit at any time with `python3 tools/bind_unit.py --show`.

## Phase 8 — record the unit

Per unit, keep alongside the config:

```
unit id                 ____________________
FC board / mcu_id       ____________________   (tools/find_fc.py)
ESP32 MAC               ____________________   (in the by-id path)
MSP ceiling measured    ______ txn/s
gyro_bias               [____, ____, ____]
accel_offset            [____, ____, ____]
accel_per_g_axis        [____, ____, ____]
mount_matrix skew       ______ deg
six-pose sign check     PASS / FAIL           date ________
tracking worst          ______ deg
armed bench test        PASS / FAIL           date ________
motor order verified    PASS / FAIL
commissioned by         ____________________
bound (bind_unit.py)    YES / NO
```

Phases 3-6 in the UI take about 8 minutes of hands-on time once practised.

---

## Per-unit configs

`config/vehicle.yaml` currently describes one airframe. For a fleet, do **not**
edit it per unit — a shared file that drifts is how a unit ends up flying
another unit's calibration.

Give each unit its own file and select it explicitly:

```
config/units/NOVA-001.yaml
config/units/NOVA-002.yaml
```

Every tool takes `--config`, and `companion.config.load(path)` takes the path,
so this needs no code change — only the discipline of never running a unit
without naming its file. Put the unit id and both board identifiers in a
comment at the top so a mismatched pairing is visible at a glance.

The FC config (`fc_diff_all.txt`) genuinely is shared and should stay one file.
Re-dump it whenever it changes: `python3 tools/fc_cli.py dump`.
