# Sign conventions on this airframe — all measured, none assumed

Every sign below was wrong at least once during bring-up, and each wrong version
produced a confident, plausible answer that passed every check available to it.
Two of them agreed with an independent sensor to a tenth of a degree while being
backwards. This file records what was measured, how, and what it cost to get
there.

**The rule this exists to enforce:** a sign is not knowable from a datasheet, a
convention, or the other axis. Measure it against something that cannot be
inferred — the operator's hands.

---

## The measured values

| Quantity | Value | How it was established |
|---|---|---|
| Betaflight pitch | **positive = NOSE DOWN** | Front pointed at the ceiling → FC reported −88.2°, held to 0.1° of spread |
| Betaflight roll | **positive = RIGHT side down** | Standard, and confirmed by the six-pose check |
| `imu.pitch_sign` | **−1.0** | Consequence of the above; the standard ZYX decode assumes positive nose-up |
| `imu.roll_sign` | **+1.0** | Textbook. Do **not** infer it from pitch |
| Nose DOWN command | pitch channel **above** 1500 | Pilot's stick full forward → 2012 |
| Roll RIGHT command | roll channel **above** 1500 | Pilot's stick full right → 2012 |
| Correct a NOSE-UP tilt | command pitch **above** 1500 | Derived from the two above |
| Correct a RIGHT-down tilt | command roll **below** 1500 | Derived — note the opposite sense |
| Pitch stick channel | `MSP_RC` index **1**, travel 989..2012 | Swept all 16 channels while only that stick moved |
| ESP32 → airframe | x←sensor y, y←sensor x, z←−sensor z, plus 7.5° | Kabsch fit against the FC, 404 samples, 0.92° residual |

`MSP_RC` returns **roll, pitch, yaw, throttle**. `MSP_SET_RAW_RC` takes raw
receiver order with throttle at index **2**. They are not the same order.

---

## The five sign errors, and why each survived its checks

### 1. Betaflight reports pitch positive NOSE DOWN

The euler decode assumed the textbook sign. Every pair fed to the axis-map fit
therefore had its x component inverted, which no rotation can reconcile — so the
fit refused with a 45° residual, four times, while the data was fine.

**Why it hid:** the failure looked exactly like a badly mounted ESP32. Comparing
the FC to its own accelerometer could not resolve it either, because a sign error
in *either* produces the identical symptom.

**What found it:** `tools/check_fc_attitude.py`, which validates against the
**pose name** rather than another sensor. The operator knows they are holding the
front at the ceiling; that is ground truth no sensor can supply.

### 2. The axis fit solved against gravity instead of specific force

An accelerometer at rest measures specific force, which points **up**. The fit
paired the sensor's reading against the FC's **down** vector, so it was solving
for the closest proper rotation to −R.

That problem is **degenerate**: −R is improper, and the nearest rotation to it is
any of infinitely many 180° rotations. The optimum is not unique, so the answer
is arbitrary.

**Why it hid:** arbitrary but *structured-looking*. It returned a clean 90° swap
that matched the operator's own read of the mounting. Only the z sign was wrong,
and only a fact known from the raw counts — the sensor reads +1 g on z when level
— exposed it.

### 3. The correction rule assumed roll and pitch were symmetric

Correcting a right-side-down tilt means rolling **left**. Correcting a nose-up
tilt means pitching the nose **down**. Opposite senses.

**Why it hid:** both sticks read HIGH at full deflection (2012 and 2012), which
makes symmetry look obviously right. It isn't — the asymmetry is in what
"correcting" means per axis, not in the channel values.

**What found it:** measuring both stick directions from the pilot's transmitter
instead of deriving one from the other.

### 4. The 3D view was drawn from underneath

The elevation term subtracted where it should have added, so the far end of the
ground grid drew *lower* than the near end and the scene read as seen from below.
From below, right-wing-down is indistinguishable from left-wing-down.

**Why it hid:** it looked like a real roll inversion, and nearly caused a correct
`roll_sign` to be flipped — which would have broken a working calibration and
invalidated step 2 with it.

**What found it:** the operator noticing that the position tiles read correctly
while the model looked mirrored. Same data, no 3D in the tiles ⇒ the data was
right and the viewpoint was wrong. That asymmetry was the whole diagnosis.

### 5. `MSP_RC` field order in the stick tool

Printed a centred yaw as "thr 1498" and a low throttle as "yaw 989". Cosmetic
here, but the same confusion in the command path would be anything but.

---

## What actually clears a sign: `bringup` → calibration UI → sign check

Six buttons, one per position. The operator holds the airframe and presses the
button; both sensors are recorded and checked against what that position
**requires**, with a green or red cell per sensor — so a failure says not only
that a sign is wrong but **whose**.

Result, 2026-08-05, all six passing on both sensors:

```
position                      ESP32 roll/pitch      FC roll/pitch
Level                          -2.9 /   4.3  OK     -2.9 /   3.5  OK
Nose up (camera at ceiling)    55.2 /  86.8  OK     41.2 /  86.8  OK
Nose down (camera at floor)    -8.5 / -81.7  OK    -12.7 / -81.7  OK
Roll right (R wing down)       83.2 /   1.7  OK     83.7 /   1.8  OK
Roll left (L wing down)       -85.9 /  -5.6  OK    -85.6 /  -5.3  OK
Inverted                     -169.7 /   1.7  OK   -168.8 /   1.9  OK
```

⚠️ **The roll figures in the two nose-vertical poses disagree by 14° and 4° while
pitch matches to the decimal.** That is **gimbal lock**, not a fault: at 87° of
pitch the Euler roll is ill-conditioned, because the nose lies nearly along the
axis roll is measured about. Both sensors describe the same physical attitude;
the coordinates stop being well defined. The filter's quaternion is unaffected.
Do not chase it and do not gate on it.

---

## Why cross-checking two sensors is not enough

The axis-map fit aligns the ESP32 to the **FC's frame**. If that frame has a sign
wrong, the fit absorbs it: the two sensors then agree beautifully and are both
wrong together. Measured evidence of exactly this state during bring-up —

- axis-map residual **0.92°**
- tracking agreement **0.66° roll / 0.78° pitch** across a 127° sweep
- static agreement **0.1°**

— all while the model was drawn inverted. Every number was excellent and one of
them was meaningless.

**A shared error is invisible to a cross-check.** Only an external reference —
the operator, a picture, a pose name — can see it.

## Terminology, because it caused real confusion

"Nose up" is ambiguous between the airframe and the stick, and the two are
**opposite**: a multirotor tips its nose DOWN to fly forward, so the "fly
forward" stick input is nose-down.

Use physical descriptions that cannot be mirrored:

- **"camera pointing at the ceiling"** — not "nose up", not "pitch up"
- **"the wing on your right when looking out through the camera"** — not "right
  side down", which depends on where the operator is standing

The calibration UI uses the airframe descriptions throughout, and labels the 3D
model's wingtips **L** and **R** so no mental rotation is required.
