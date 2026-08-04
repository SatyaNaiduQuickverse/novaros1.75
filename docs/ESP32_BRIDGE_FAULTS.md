# ESP32 IMU bridge: measured failure modes

Everything here was measured on this airframe on 2026-08-04, not inferred from
datasheets. Two faults cost most of a day between them, and both are the kind
that look like nothing is wrong:

1. **The bridge can be enumerated and completely silent, indefinitely.**
2. **The accelerometer reports a different `|a|` depending on which way it
   faces** — so a single `accel_per_g` cannot describe it.

Read this before debugging "the IMU stopped" or "the attitude is off by a few
degrees", and before changing `companion/imu_esp32.py` or `esp32/main.py`.

---

## 1. The silent bridge

### Symptom

`/dev/serial/by-id/usb-Espressif_*` is present. `dmesg` is clean. No errors
anywhere. The port opens fine. And **zero bytes arrive** — measured once for
19 minutes straight, ended only by a chip reset.

Downstream this is worse than a crash: the reader hands out its last sample
forever, so the control loop runs happily on a **frozen attitude**. Nothing
raises. `stale()` is the only thing that knows.

### Why presence proves nothing

On the ESP32-C6 the USB-Serial-JTAG is a **hardware peripheral**. It enumerates
whether or not the firmware is alive, whether or not `main.py` ever ran.

> Seeing the device under `/dev/serial/by-id/` tells you the chip has power.
> It tells you nothing about whether anything is running on it.

### Cause: nobody was holding the port open

Two measured facts combine:

- **Linux `cdc_acm` only submits read URBs while the tty is open.** With no
  process holding the port, the device's IN endpoint is never drained.
- **DTR gates the stream.** Six for six across 115200 and 921600, on a
  freshly-opened handle: DTR high → 3618 B/s, DTR low → 130 B/s (the tail of
  what was already buffered) and then nothing. Linux deasserts DTR whenever no
  process holds the tty open.

  *Scope, honestly:* this did **not** reproduce once the reader already held
  the port and had reset the board — dropping DTR there left the stream running
  at 200 Hz. Treat it as "an unheld port means a silent bridge", not as a live
  kill switch.

So an idle port is indistinguishable, from the bridge's point of view, from an
absent computer. That is exactly the state the Pi boots into.

The timeline from the incident, straight out of `dmesg -T`:

```
20:08:58  usb 1-2: new full-speed USB device        <- ESP32 powers up
20:09:02  cdc_acm 1-2:1.0: ttyACM0: USB ACM device  <- Pi's driver attaches, 4s later
```

The bridge began streaming ~2 s after boot, into a port nothing had opened,
blocked on its first writes, and never recovered.

### Fix: open the port first, then reset the chip into it

Implemented in `companion/imu_esp32.py`:

```python
imu = ESP32IMU(...).start()     # opens the port, THEN pulse_reset(), then reads
```

`pulse_reset(ser)` takes an **already-open handle**. That ordering is the whole
fix, not an implementation detail — rebooting the bridge into a port we already
hold makes "the companion started late" impossible by construction, rather than
something the firmware has to get lucky about. Reset-then-open recreates the
original race exactly, and there is a test asserting the order.

Costs ~2.7 s at startup; `start()` blocks in `wait_ready()` until frames flow.
Pass `reset=False` only to attach to a bridge already known to be streaming.

The reset itself: `DTR=False` (GPIO9 high ⇒ normal flash boot, **never** the ROM
download mode), `RTS` pulse (EN), `DTR=True`, then a wake byte.

> ⚠️ **`write_timeout` is mandatory on any handle to this device.** The firmware
> currently on the board cannot drain its OUT endpoint, so an unbounded write
> blocks forever. This deadlocked `start()` during development.

### Recovery while running

The reader watches for **silence, not slowness** — the bridge emits 3600 B/s or
nothing, so there is no middle ground to threshold. After `recover_after_s`
(default 3.0) it escalates:

1. `_nudge()` — re-assert DTR, wait 0.4 s. No reboot, no outage.
2. Full `pulse_reset()` — reopen, reboot, ~3 s of no attitude.

Both counted in `stats()` as `nudges` / `recoveries`. **Watch these in flight
logs**: a rising `recoveries` means the bridge is unhealthy even though every
other number looks perfect.

Hardware-verified by rebooting the board out from under a running reader:

```
external chip reset injected
  t+ 1s  frames=1412  stale=True
  WARNING: ESP32 bridge silent for 1.5s — resetting (recovery #1)
  t+ 5s  frames=1500  stale=False      <- back at 200 Hz
  t+12s  frames=2901  recov=1  drops=0
```

Two traps, both hit for real, both now covered by tests:

- **Clear `_last_seq` on recovery.** The bridge restarts its sequence counter at
  0; without clearing, that discontinuity is billed as 68 dropped samples and a
  clean recovery reads as a link problem.
- **Hold a `BOOT_GRACE_S` window (14 s) after any reset.** The bridge is
  legitimately silent while it boots. Without the grace the reader calls that a
  wedge, resets it again, and loops forever — the recovery becomes the outage.

### The firmware cannot be interrupted while streaming

Measured: once `stream()` is running, **every** host→device write times out. So
Ctrl-C never lands, `mpremote` cannot connect, and `STARTUP_DELAY_S` buys
nothing. The 200 Hz loop starves MicroPython's USB-Serial-JTAG RX task.

The only ways in are the ROM download mode (esptool, which is independent of
firmware) or a full erase and reflash.

`esp32/main.py` in this repo already contains the fixes for the next flash:

| change | why |
|---|---|
| `STARTUP_DELAY_S = 10.0` | do not stream before a host could plausibly be listening |
| `_wait_for_host()` polls stdin | any byte ⇒ start now; `Ctrl-C` ⇒ REPL. A ceiling, not a fixed cost |
| `WDT(4000)`, fed **after** the write returns | a blocked write is what must trip it; feeding first would hang politely |
| `time.sleep_ms(0)` yield in the loop | lets the RX task drain the OUT endpoint, so Ctrl-C can land |
| WDT **not** started if a break was requested | an ESP32 WDT cannot be switched off once running; starting it unconditionally would make the board permanently unreflashable except via esptool |

> ⚠️ **The device is still running the older build.** Everything in the table is
> correct for the next flash and must not be read as a description of the board
> currently on the bench. The Pi-side fixes above work regardless of which
> firmware is loaded, which is why this is not blocking.

---

## 2. The accelerometer is not single-scale

### Symptom

`|a|` at rest should be exactly 1 g in **every** orientation — that is the whole
basis of "accel scale needs no level surface". This part does not do that.

Three poses, all stationary to within 4 counts of noise:

| pose | gravity on | raw counts | `\|a\|` | vs nominal 2048 |
|---|---|---|---|---|
| A | sensor z | `[  61.9, -102.0, 2397.2]` | 2400.2 | +17.2% |
| B | sensor x | `[-1953.8,  314.0, -268.1]` | 1996.9 | −2.5% |
| C | sensor z | `[ -79.9, -259.7, 2382.9]` | 2398.3 | +17.1% |

Poses A and C were taken an hour apart in different physical orientations and
agree to 0.1%. The result tracks **which axis gravity lands on** — not time, not
temperature, not drift. It is a fixed, repeatable property of the die.

Consistent with a per-axis zero-g offset of roughly **+350 counts (0.17 g)** on
sensor z — well outside the MPU6500's spec, and typical of the dies on these
cheap modules. The stored `accel_per_g: 2077.5` was simply whatever one pose
happened to give.

### Why this matters more than it looks

The Mahony filter **normalises** the accelerometer vector before using it:

```python
a_hat = a / np.linalg.norm(a)
```

So the two error types are not remotely equivalent:

- A common **scale** error cancels out completely. Invisible to attitude.
- A per-axis **offset** tilts the measured gravity *direction*. It becomes
  standing attitude error, degree for degree.

A 350-count offset against ~2000 counts of gravity is about **10 degrees**. The
self-level module's entire job is to fly toward what it believes is level — tell
it level is 10 degrees off and it will hold the airframe 10 degrees off, with
complete confidence and no error anywhere.

There is a test that computes this number from the measured figures
(`test_uncorrected_offset_is_a_real_attitude_error`) so the consequence stays
attached to the fact.

### Fix: per-axis offset and scale

```yaml
imu32:
  accel_offset:      [ox, oy, oz]      # raw counts, SENSOR frame
  accel_per_g_axis:  [sx, sy, sz]      # counts per g, per axis
  accel_per_g:       2077.5            # fallback only, unused once the above is set
```

Applied in the **sensor** frame, before `axis_map` — offset and scale are
properties of the die, `axis_map` is a property of how it is bolted in:

```python
accel = self._to_body((raw_a - self.accel_offset) * (G / self.accel_scale))
```

### Measuring it: tumble, not six flat faces

```bash
make calib-ui                                           # browser UI, recommended
python3 -m tools.bringup imu32 --accel-cal              # same thing, CLI
python3 -m tools.bringup imu32 --accel-cal --poses      # six flat faces
```

**Prefer the UI.** The CLI asks the operator to turn the airframe for ninety
seconds and only reports at the end whether the coverage was sufficient — two
runs were wasted that way on an airframe nobody had picked up. The UI shows
live which of the six directions are still missing, whether the airframe is
currently still enough for a sample to count, and a plot of what has been
collected so gaps are visible rather than inferred. It serves from the Pi
because it has to hold the serial ports open; a sandboxed page cannot.

Six-face calibration assumes you can rest the thing squarely on each face. The
IMU is bolted into an airframe; you cannot. So the default fits an **ellipsoid**
instead, which needs no pose to be square to anything:

> Every at-rest sample lies on a sphere of radius 1 g, whichever way the sensor
> points. A real part smears that sphere into an ellipsoid displaced from the
> origin. Its **centre is the offset** and its **radii are the per-axis scales**.

Solve `A·a² + B·a = 1` (six unknowns, absorbing the constant), then:

```
o_i   = -B_i / (2 A_i)
D     = 1 / (1 + Σ o_i² A_i)
s_i   = 1 / sqrt(A_i · D)
```

Procedure: pick the airframe up and rotate it slowly through as many
orientations as you can for ~90 s, **pausing about a second in each**. Only
samples where the bias-corrected gyro reads under 8 dps are kept, because the
fit assumes gravity is the only specific force — waving it about adds real
acceleration and poisons the answer. Move-and-pause, not wave.

> ⚠️ **An ellipsoid can be fitted through any patch of its own surface.** A cloud
> confined to one octant yields a confident, precise, completely wrong centre.
> The fit therefore **refuses** unless every axis swung through ≥60% of ±1 g,
> and `_imu32_tumble_cal` refuses earlier with a friendlier message. This
> calibration decides which way is down; guessing is not an acceptable failure
> mode. Tests cover both the one-octant cloud and a single-axis-of-rotation
> spin.

### Signs

Everything above establishes the MAGNITUDE of the calibration. The signs are a
separate problem and a worse one — see **[SIGN_CONVENTIONS.md](SIGN_CONVENTIONS.md)**,
which records five sign errors from this bring-up, each of which agreed with
every cross-check available to it while being backwards.

### Order matters

```
--accel-cal   THEN   --axis-map
```

`--axis-map` decides each axis and sign from whichever raw component is largest.
An 0.17 g offset is easily enough to mis-call a shallow tilt — and a wrong sign
there makes the self-level loop drive **into** the tilt rather than out of it.
`--axis-map` subtracts `accel_offset` if it is configured, and warns loudly if
it is not.

---

## Operator notes that cost time

- **`bringup imu32` reporting `|a| = 9.44` instead of 9.81** is this fault, not
  a bug. Once `accel_per_g_axis` is set it reads 9.81 in every pose.
- **Watch the "turned so far" numbers during `--accel-cal`.** All three must
  climb past 0.70g. If they sit near zero, the airframe is not actually moving
  and the run is wasted — stop it.
- **A calibration progress meter must be seeded from the data, never from
  zeros.** Seeded at 0, the per-axis max/min measured *distance from the origin*
  rather than *how far it turned*, so a motionless board with gravity at −1954
  counts reported "0.48 g of coverage" and two 90-second runs of an airframe
  nobody picked up looked half done. Pinned by
  `test_coverage_of_a_motionless_board_is_near_zero`.
- **Before blaming the operator, prove the sensor is live.** "Nobody moved it"
  and "the reader is frozen" produce identical logs. Sample raw counts and check
  the noise: a live MPU6500 at rest shows 2–4 counts of standard deviation and
  changes on nearly every sample. Zero variance means frozen.
