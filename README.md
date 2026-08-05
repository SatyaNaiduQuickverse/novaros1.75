# Companion connector layer

The plumbing between a Raspberry Pi 5 companion computer and a Betaflight
flight controller: MSP link, attitude/IMU, camera-to-world bearings, logging,
and a bench harness that verifies each path on its own.

A companion computer reads vehicle state and an object bearing from a
camera/tracker and streams stick commands to the FC over MSP override. This
repository is the connector layer only — `companion/command_module.py` is a
placeholder that emits an idle command, and gets replaced by the operational
module once every path below is verified.

## Layout

```
companion/            the library
  msp.py              MSP v1 client — framing, CRC, jumbo frames, thread-safe
  safety.py           AETR channel order + the command envelope
  config.py           ports, switch map, calibrations, limits
  fc_link.py          FCLink: telemetry + safety-clamped override streaming
  imu_driver.py       attitude/IMU providers (FC-backed, fake, dedicated)
  vision_adapter.py   tracker bbox -> world-frame bearing
  vision_interface.py the Bearing type
  math_utils.py       quaternions (Hamilton, body FRD -> world NED)
  flight_logger.py    per-tick JSONL logging
  command_module.py   per-tick command producer (idle placeholder)
config/vehicle.yaml   everything vehicle-specific, in one file
tools/bringup.py      the bench harness
tools/log_analyze.py  run-log summary
tests/                offline tests — no hardware required
reference/            the original unmodified kit, for diffing
```

## Quick start

```bash
python3 tools/find_fc.py   # identify the FC after any board swap
make test          # 169 offline tests, no hardware
make loop-fake     # the whole loop with fake IMU, fake vision, no FC
make preflight     # with the FC plugged in: who is on the port
```

**Building a unit?** [docs/COMMISSIONING.md](docs/COMMISSIONING.md) is the
ordered runbook — FC, then bridge, then calibrate, then verify, then the armed
bench test, with the gates that stop a unit that passes every check and still
flies wrong. It also states plainly what transfers between units and what must
be re-measured for each one, which is the difference between a 40-minute build
and an aircraft calibrated for a different airframe.

Then work through [docs/SETUP_HARDWARE.md](docs/SETUP_HARDWARE.md) for the
detail behind any step.

Signs — attitude, correction direction, channel values — are measured on this
airframe, never assumed. [docs/SIGN_CONVENTIONS.md](docs/SIGN_CONVENTIONS.md)
records what was measured and the five sign errors that got there, each of
which passed every check available to it while being wrong.

If the ESP32 IMU is involved at all, read
[docs/ESP32_BRIDGE_FAULTS.md](docs/ESP32_BRIDGE_FAULTS.md) first. It documents
two measured faults that both present as "nothing is wrong": a bridge that is
enumerated and permanently silent, and an accelerometer whose reported gravity
depends on which way it faces. Each one silently produces a plausible attitude
that is simply incorrect.

## How control authority works

Betaflight's `msp_override` lets the companion replace selected RC channels
while the pilot's radio keeps the rest. On this airframe:

| Situation | Who has the sticks |
|---|---|
| Override switch (ch8) down | Pilot transmitter, entirely |
| Override switch up | Companion drives ch1-4; pilot keeps arm, modes, and the switch |
| Companion stops streaming | FC drops the override in ~250 ms, pilot regains control |

That last row is the safety architecture. **Every abort path in `FCLink` works
by stopping the stream**, so a crashed process, a wedged loop, an unplugged USB
cable and a deliberate abort all end the same way. Measured pilot recovery on
this airframe when flipping the switch off: ~53 ms.

Requires firmware built with the MSP Override option — box id 50 must appear in
`MSP_BOXIDS` or `FCLink.connect()` refuses to run. Stock SPEEDYBEEF7V3 builds
lack it.

Three rules are enforced in code for any module that goes through the documented
seam:

- `FCLink.arm()` raises. The companion never arms; ARM is on ch9 and belongs to
  the pilot. `msp_override_channels_mask = 15` means aux channels are not
  overridable anyway — the code just refuses to try.
- Only four channels are ever transmitted. `safety.aetr_frame()` is the single
  place channel bytes are packed.
- Throttle and deflection are clamped when the frame is built, so no caller
  mistake can exceed the envelope. A NaN or infinity degrades to centred sticks
  at the throttle floor rather than raising inside the stream thread.

**Those three are cooperative, not mandatory.** Any code inside this process —
including a compiled command module — can call `fc.msp.request(MSP_SET_RAW_RC,
...)` directly and bypass all of them. What survives regardless is enforced by
the flight controller: `msp_override_channels_mask = 15` keeps ARM with the
pilot, the pilot's switch, and the dead-man timeout. As partial cover,
`fc.verify_wire` (on by default) reads `MSP_RC` back during each watchdog check
and aborts if what the FC actually received is outside the envelope — detection
at ~3 Hz, not prevention. Before loading a third-party or compiled module, read
[docs/COMMAND_MODULE_INTERFACE.md](docs/COMMAND_MODULE_INTERFACE.md) and run:

```bash
python3 tools/check_module_interface.py <module> --path <dir>
```

## The one API gotcha

`MSP_SET_RAW_RC` (cmd 200) takes channels in **raw receiver order**, which with
the standard AETR map puts throttle at index 2:

```
index    0      1       2         3
meaning  ROLL   PITCH   THROTTLE  YAW
```

`MSP_RC` (cmd 105) *returns* roll, pitch, yaw, throttle — throttle last.
Confusing the two once streamed 1550 onto the throttle channel of this
airframe. Nothing outside `safety.aetr_frame()` packs channel bytes, and
`tests/test_safety.py` pins the ordering.

## Configuration

Everything vehicle-specific lives in `config/vehicle.yaml`; the defaults in
`companion/config.py` apply when a key is absent. Two calibration blocks ship
marked `verified: false` — the IMU axis map and the camera intrinsics/boresight.
Both silently bias every bearing until measured, so `preflight` lists them until
they are cleared. `bringup calib-imu` measures the IMU block and prints the YAML
to paste back.

## Safety

Props off for the entire bring-up sequence. A pilot with a transmitter must be
able to take control instantly at all times, and that is the layer everything
here runs behind — test it physically before each session, not just in software.
Remove one safety layer at a time when progressing to powered, tethered, then
flight tests, and raise `limits.profile` from `bench` only when the step below
it is green.
