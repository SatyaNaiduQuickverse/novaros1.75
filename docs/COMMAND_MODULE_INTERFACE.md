# Command module interface contract

The seam an operational module must satisfy to replace
`companion/command_module.py`. Verify a compiled or third-party module against
the exact signatures below — matching the shape of `(fc, imu, vision).step(t)`
is necessary but not sufficient, because the argument semantics (units, frames,
clock epoch, nullability) are where a mismatch is silent rather than loud.

Verify with `python3 tools/check_module_interface.py <module.path>`.

## Construction and lifecycle

```python
CommandModule(fc, imu, vision, logger=None)      # logger may be None
CommandModule.engage()                           # once, when control is taken
CommandModule.step(t: float) -> dict             # every control tick
```

`engage()` is called at the moment the companion takes control, and starts the
module's phase clock. `step()` must not block: it runs inside the control loop
at `fc.cfg.fc.rc_hz` (currently 35 Hz, a 28.6 ms budget), and overrunning it
directly slows the RC stream. It is called from the loop thread, not the RC
stream thread.

### `t` is seconds since `engage()`

Taken from `time.monotonic()` — NTP-safe — but **offset so it starts at ~0 at
engagement**. The correct call pattern is:

```python
module.engage()
t0 = time.monotonic()
while running:
    module.step(time.monotonic() - t0)
```

Capture `t0` at the same moment as `engage()`, not at process start or loop
construction.

**This is a silent-failure interface, which is why it gets its own section.** A
module that sequences time-based phases off `t` — a boost phase, a terminal
phase — will mis-sequence if handed any other epoch, without raising anything.
Hand it an absolute monotonic clock (uptime, so possibly days) or
seconds-since-process-start and its first tick arrives with `t` already large,
so it starts in a late phase. The failure is not a crash; it is the guidance
doing the wrong thing confidently.

`CommandModule._check_epoch()` warns if the first `t` exceeds 1 s or if `t`
goes backwards. `check_module_interface.py` sweeps `t` across a full simulated
engagement for the same reason — see below.

### `Bearing.t` shares whatever epoch `step()` was given

`step(t)` passes its own `t` straight to `vision.bearing(t)`, which stamps it
onto `Bearing.t`. So within a run, measurement age is `t - b.t` — but the
absolute value is only meaningful relative to engagement. A module that uses
each bearing at the current step (an EKF update, say) never needs `b.t` at all.

The return value is a dict of the sticks commanded — used only for logging and
statistics. **The module must call `fc.set_stick()` itself**; returning a dict
does not command anything.

## `fc` — `companion.fc_link.FCLink`

```python
fc.set_stick(roll=None, pitch=None, yaw=None, throttle=None) -> None
```

Keyword arguments, microseconds, int or float. `None` leaves that channel at
its previous value. Values are clamped when the frame is built, so asking for
out-of-range is safe but silently truncated — check `fc.limits` rather than
assuming your request went through verbatim.

**Omitting `throttle` is a trap.** With `msp_override_channels_mask = 15` the
FC uses the *companion's* throttle, not the pilot's stick. A module that steers
attitude and never commands throttle therefore freezes throttle at whatever was
last sent, and **the pilot's throttle stick cannot reduce it** — their only
remedy is the override switch. Measured on this airframe: after a no-throttle
call the FC held 1050 µs while the pilot's stick sat at 989 and was ignored.

Either command throttle every tick, or set `msp_override_channels_mask = 11`
(`0b1011` — ch1, ch2, ch4) so ch3/throttle stays with the pilot and the
companion only supplies attitude. `FCLink` warns after
`THROTTLE_STALE_S` (2 s) of unrefreshed throttle under an active override.
Re-sending the same constant value every tick is fine and never trips it.

Read-only state (each costs one MSP transaction — see the budget note below):

```python
fc.read_attitude() -> {"roll": deg, "pitch": deg, "yaw": deg} | None   # cached, free
fc.read_imu()      -> (accel_frd, gyro_frd) | None                     # cached, free
fc.attitude_q()    -> np.ndarray (4,)      # Hamilton (w,x,y,z), body FRD -> world NED
fc.armed()         -> bool                 # 1 transaction
fc.override_active() -> bool               # 1 transaction
fc.rx_link_up()    -> bool                 # 1 transaction
fc.motors()        -> list[int]            # 1 transaction, 4 values
fc.rc()            -> list[int]            # 1 transaction; roll,pitch,YAW,THROTTLE,aux...
fc.abort_reason    -> str | None           # attribute, free
fc.limits          -> Limits               # attribute, free
```

`fc.arm()` raises `PermissionError` by design. ARM is ch9 and belongs to the
pilot; `msp_override_channels_mask = 15` means aux is not overridable anyway.

`read_attitude()` and `read_imu()` return the telemetry thread's cache and cost
nothing. They return `None` before the first successful poll — handle it.

## `imu` — `get_state()`

```python
q, w, accel_body = imu.get_state()
```

| | type | units | frame |
|---|---|---|---|
| `q` | `np.ndarray (4,)` float64 | Hamilton, scalar first `(w,x,y,z)` | rotates body FRD → world NED |
| `w` | `np.ndarray (3,)` float64 | rad/s | body FRD |
| `accel_body` | `np.ndarray (3,)` float64 | m/s² specific force | body FRD |

Body FRD is x forward, y right, z down. World is NED. Level and at rest,
`accel_body` reads `[0, 0, -9.81]` — specific force points *up*.

Never returns `None`; `FCIMU` holds its last good sample if telemetry stalls,
so check `imu.stale()` if freshness matters.

## `vision` — `bearing(t)`

```python
b = vision.bearing(t)     # -> Bearing | None
```

**Returns `None` whenever there is no confident measurement** — that is the
normal case, not an error, and a module that assumes a `Bearing` will crash
within seconds of a real tracker losing lock.

```python
b.u_world  # np.ndarray (3,) float64, UNIT vector, world NED
b.range_m  # float | None — size-based hint; None when the box is too small
b.conf     # float, 0..1
b.t        # float, time.monotonic() seconds of the FRAME the box came from
```

`b.range_m` is `None` below `camera.min_box_px` (26 px) because a one-pixel
error swings the estimate wildly there. Do not treat `None` as zero.

## `logger` — `companion.flight_logger.FlightLogger | None`

```python
logger.log(**fields) -> dict      # one JSONL record per tick
logger.event(name, **fields)      # named event, e.g. "lock", "terminal"
```

numpy arrays, floats, lists and dicts are encoded automatically.

## Constraints a module must respect

**Never write aux channels.** Only `set_stick()`'s four channels.

**Do not exceed the MSP transaction budget.** The FC serves a fixed number of
transactions per second *total*, shared with the RC stream, telemetry and the
watchdog — and **it is a per-board property, not a constant**. Measured on this
Pi: F722 / BTFL 26.6.1 served ~99/sec (10.1 ms per round-trip); F405 / BTFL
4.5.1 serves ~62/sec (16.05 ms). Rates that were comfortable on one board
oversubscribe the other by a third. `cfg.fc.msp_txn_per_sec` holds the measured
value and `FCLink.txn_budget()` does the arithmetic against it. A module that
polls `fc.armed()` or `fc.rc()` every tick adds one transaction per tick and
will push the RC stream below its configured rate — silently, with no error. Use the free cached accessors
(`read_attitude`, `read_imu`, `abort_reason`) in the tick path and poll the
transactional ones at a few Hz at most.

**Respect `fc.abort_reason`.** Once set, the stream has stopped and the pilot
has control. Keep computing if you like, but do not attempt to re-establish
the stream.

## Trust boundary — read this before loading a compiled module

The clamps in `safety.aetr_frame()` are **cooperative, not enforced**. Any code
inside this process — including a compiled extension — can call
`fc.msp.request(MSP_SET_RAW_RC, ...)` directly and bypass every clamp and the
AETR ordering, or rebind `fc.limits` to a wider profile in one statement. The
`Limits` dataclass being frozen prevents field mutation, not rebinding.

What survives regardless, because it is enforced by the flight controller:

- `msp_override_channels_mask = 15` — aux genuinely not overridable, so **ARM
  stays with the pilot** no matter what the module does.
- The pilot's physical override switch.
- The ~250 ms dead-man timeout: a module that hangs or crashes fails safe,
  because the stream stops and the pilot recovers control.

Partial mitigation in this codebase: `fc.verify_wire` (on by default) makes the
watchdog read `MSP_RC` back and abort if what the FC *actually received* is
outside the envelope. Verified on hardware — it caught a deliberate raw-MSP
bypass of `roll=1900, throttle=1400`.

**This is detection, not prevention.** It samples at `watchdog_hz` (3 Hz), so a
rogue frame is acted on for up to ~333 ms before the tripwire fires. For real
prevention, run the module out-of-process behind an IPC boundary so it can only
express intent, and/or set FC-side limits underneath it so the ceiling is
enforced in firmware rather than in Python.

### What the checker can and cannot certify

It exercises the module through the documented seam against a mock FC, sweeping
`t` across a simulated engagement so time-based phases actually execute. It
cannot prove the absence of a bypass on a path it did not exercise. Two
concrete limits worth stating to whoever signs off:

- **Set `--duration` above the module's longest phase.** The default sweep is
  30 s. If terminal phase begins at 40 s, a default run certifies everything
  *except* terminal.
- **It validates behaviour, not the artifact.** Passing tells you *a* module
  behaved; it does not attest that the `.so` on the vehicle is the one that was
  certified. If the binary is the trust anchor, hash it at build time and check
  the hash at load — behavioural testing cannot substitute for provenance.

## Out-of-process isolation — design constraints

Moving the module to a subprocess is what turns the cooperative clamps back
into enforced ones, because the guidance then has no serial handle at all. Two
things will silently defeat it:

**1. Do not `fork`.** On Linux, `multiprocessing`'s default start method is
`fork`, and a forked child inherits the parent's open file descriptors —
including the FC's. Measured on this Pi (Python 3.13.5, default `fork`):

```
parent holds the FC on fd 3, rdev 0xa600 (/dev/ttyACM0)
  fork   -> CAN REACH THE FC     fd 3: chardev rdev=0xa600 (parent 0xa600)
  spawn  -> cannot reach the FC  fd 3: not a chardev rdev=0x0
```

A forked guidance process can write raw MSP to the flight controller while
appearing fully isolated. Use `mp.get_context("spawn")`, or `subprocess` with
`close_fds=True`, and confirm it — checking that a file descriptor number is
*valid* in the child proves nothing, because fd numbers are reused. Compare
`os.fstat(fd).st_rdev` against the parent's and require `S_ISCHR`.

Forking is doubly wrong here anyway: the connector process is multi-threaded
(RC stream plus telemetry), and forking a multi-threaded process is hazardous
in its own right — Python 3.12+ emits a `DeprecationWarning` for exactly this.

**1b. `spawn` re-imports `__main__` in the child — guard your entry point.**
Switching to `spawn` fixes fd inheritance but introduces a second, subtler way
to leak the port. `multiprocessing.spawn` re-executes the parent's `__main__`
module in the child to make pickled references resolvable. If that module opens
the serial port at import scope, **the child opens its own handle to the flight
controller** — no inheritance required. Hit during development of this
handler: the child re-ran the launching script, opened the FC, and collided
with the parent's link (`MSPTimeout` on cmd 119).

Every entry point that constructs an `FCLink` must therefore sit under
`if __name__ == "__main__":`, with the port opened inside a function rather
than at module scope. `tools/bringup.py` complies.

**1c. Have the child prove its own isolation.** Both leaks above are invisible
from the connector side, so `guidance_proxy._child_entry` checks before any
guidance code is imported: it scans `/proc/self/fd` (skipping stdio, since a
controlling terminal is a character device too) for anything under
`/dev/ttyACM|ttyUSB|ttyAMA|ttyS|serial/`, and calls `os._exit(90)` if it finds
one. The parent sees EOF and refuses to run. Fails closed, catches both causes,
and costs nothing.

## The connector-side boundary handler

`companion/guidance_proxy.py` implements the trusted half. It presents the
ordinary `(fc, imu, vision).step(t)` seam, so nothing else in the stack changes:

| Guarantee | Mechanism |
|---|---|
| Binary is the certified one | connector hashes the `.so` itself *and* compares the host's reported digest; refuses on mismatch or if `cert_sha256` is unset |
| Guidance cannot reach the FC | `spawn` + child-side fd probe, fails closed |
| Clamps are load-bearing | `set_stick` → `aetr_frame` runs in the port-owning process, on data arriving over IPC |
| Hung guidance fails safe | `poll(poll_timeout_s)`, default 5 ms, then abort |
| Dead guidance fails safe | parent closes its copy of the child pipe end so EOF is detectable; also checks `is_alive()` before each send |
| Malformed reply fails safe | reply shape and tag validated before use |
| Nothing inherited across runs | `engage()` commands `IDLE_STICKS` before the first tick |

Measured end-to-end against the live FC, guidance demanding
`(2400, 300, -800, 2000)`: the FC received `[1600, 1400, 1400, 1100]` — exactly
the envelope. Abort latencies: hung host 28 ms, host death 23 ms, protocol
violation 22 ms. IPC round-trip mean 0.09 ms, worst 0.27 ms against a 20 ms
tick budget.

`tools/fake_guidance_host.py` is a reference host implementing the protocol,
with fault injection (`NOVA_HOST_FAULT=hang|die|badproto|badsha|overrange`) for
exercising each abort path without the real binary.

**2. Clamp on the connector side of the boundary.** The guidance process emits
*desired* sticks; `aetr_frame()` must run in the process that owns the serial
port, on data arriving over IPC. Clamping inside the guidance process puts the
enforcement back inside the thing being constrained.

Latency is not a concern at these rates: the loop budget is 20 ms at 50 Hz and
a shared-memory or localhost round-trip on a Pi 5 is well under 1 ms.
