"""FCLink — companion-computer control link to a Betaflight flight controller.

Control model (Betaflight ``msp_override``): the companion streams channels 1-4
over MSP while the pilot's transmitter keeps everything else. Authority at any
instant:

    override switch DOWN  -> pilot TX controls everything
    override switch UP    -> companion controls roll/pitch/yaw/throttle;
                             pilot still owns arm, modes, and the switch itself
    stream stops          -> FIRMWARE DEPENDENT. See below. Do not assume.

**The override timeout is not universal, and this board does not have one.**

Measured F722 / BTFL 26.6.1: frames stop -> override times out in ~250 ms and
the pilot gets control back automatically. The original design leaned on that.

Measured F405 / BTFL 4.5.1 (2026-08-04, the current board): **no timeout at
all.** Five seconds after the stream stopped, the FC was still applying
roll 1560 / throttle 1080 and ignoring the pilot's sticks entirely. It also
retains those values across companion process restarts — the buffer lives on
the FC. Arming with the override switch up therefore applies whatever was left
in that buffer, immediately.

Consequences, which drive the design here:

  * :meth:`_abort` does NOT stop the stream. It enters safe mode and keeps
    streaming centred sticks at the throttle floor, because that is the safest
    thing this process can still assert.
  * :meth:`close` sends a final idle frame so the buffer is left safe.
  * If this process dies without closing (kill -9, USB unplugged), the FC holds
    the last command. Nothing on the companion side can fix that.
  * **The pilot's override switch is therefore the only cutout that always
    works.** Verified on this board: switch down -> ch1-4 revert to the pilot.

FC prerequisites (saved to EEPROM, see docs/SETUP_HARDWARE.md):
    set msp_override_channels_mask = 15     # channels 1-4 only
    aux <slot> 50 3 1700 2100 0 0           # box id 50 = MSP OVERRIDE

Typical use:

    fc = FCLink().connect()
    fc.start_telemetry()
    fc.start_rc_stream()          # streams idle until told otherwise
    fc.set_stick(roll=1520, throttle=1050)
    ...
    fc.close()
"""

from __future__ import annotations

import atexit
import logging
import os
import signal
import threading
import time

import numpy as np

from . import msp as msp_mod
from .config import Config, IMUCal, load as load_config
from .math_utils import euler_to_q
from .msp import (
    MSP,
    MSPError,
    MSP_ATTITUDE,
    MSP_MOTOR,
    MSP_RAW_IMU,
    MSP_RC,
    MSP_SET_RAW_RC,
    MSP_STATUS,
    MSP_MODE_RANGES,
    MSP_BOXNAMES,
    BOX_ARM_PERMANENT_ID,
    BOX_MSP_OVERRIDE_PERMANENT_ID,
)
from .safety import (
    ABORT_ENVELOPE,
    ABORT_LINK_ERROR,
    ABORT_MOTOR_CAP,
    ABORT_OVERRIDE_RELEASED,
    ABORT_PILOT_DISARMED,
    ABORT_REQUESTED,
    CENTER_US,
    IDLE_STICKS,
    Limits,
    aetr_frame,
)

log = logging.getLogger("companion.fc")

G = 9.80665
GYRO_LSB_2000DPS = 1.0 / 16.4
RXLOSS_BIT = 2  # bit in armingDisableFlags meaning "no RC signal"

# Betaflight services MSP from a scheduled task, so request/response round-trips
# are quantised to that task's period rather than to USB latency. Measured on
# this airframe (F7 V3 / BTFL 26.6.1, USB VCP) 2026-08-04: every transaction
# costs ~10.1 ms, giving a hard ceiling of ~99 transactions per second SHARED
# across the RC stream, telemetry and the watchdog. Exceeding it does not
# error — the threads simply contend and every rate silently degrades.
MSP_TXN_PER_SEC = 99.0   # fallback only; cfg.fc.msp_txn_per_sec is authoritative
MSP_BUDGET_WARN = 0.90  # warn once demand passes this fraction of the ceiling
THROTTLE_STALE_S = 2.0  # unrefreshed throttle under override is a design smell


class FCLink:
    """MSP link + safety-clamped RC override streamer."""

    def __init__(self, config: Config | None = None, port: str | None = None,
                 serial: str | None = None, baud: int | None = None,
                 limits: Limits | None = None, require_override: bool = True):
        """``serial=`` is accepted as an alias for ``port=`` for compatibility
        with the original bring-up harness."""
        self.cfg = config or load_config()
        self.port_hint = port or serial or self.cfg.fc.port
        self.baud = baud or self.cfg.fc.baud
        self.limits = limits or self.cfg.limits
        self.channels = self.cfg.channels
        self.imu_cal = self.cfg.imu
        self.require_override = require_override
        self.verify_wire = self.cfg.fc.verify_wire

        self.msp: MSP | None = None
        self.arm_bit: int | None = None
        self.override_bit: int | None = None

        self._sticks = dict(IDLE_STICKS)
        # Arm intent, streamed every frame when companion arming is enabled.
        # False is the resting state and everything forces it back here.
        self._arm_cmd = False
        self._slock = threading.Lock()
        self._stop = threading.Event()
        self._rc_thread: threading.Thread | None = None
        self._telem_thread: threading.Thread | None = None
        self._tlock = threading.Lock()
        self._att = None
        self._imu = None
        self._engaged = False       # have we ever seen armed + override active?
        self._throttle_set_at = None
        self._throttle_warned = False
        # Set on abort. The stream KEEPS RUNNING in this state, forcing idle —
        # see _abort() for why going silent is not safe on this firmware.
        self._safe_mode = False
        self._hooks_installed = False
        self._closed = False
        self.abort_reason: str | None = None
        self.frames_sent = 0

    # ------------------------------------------------------------- lifecycle

    def connect(self) -> "FCLink":
        port = msp_mod.resolve_port(self.port_hint)
        self.msp = MSP(port, self.baud, timeout=self.cfg.fc.timeout_s)
        ids = self.msp.box_ids()
        if BOX_MSP_OVERRIDE_PERMANENT_ID not in ids:
            if self.require_override:
                self.msp.close()
                raise MSPError(
                    "this firmware has no MSP OVERRIDE (box id 50) — flash a "
                    "Betaflight cloud build with the MSP Override option; stock "
                    "SPEEDYBEEF7V3 builds lack it"
                )
            log.warning("firmware has no MSP OVERRIDE — companion sticks will be ignored")
        else:
            self.override_bit = ids.index(BOX_MSP_OVERRIDE_PERMANENT_ID)
        # Flag-bit positions follow this build's box order, so resolve, never hardcode.
        self.arm_bit = ids.index(BOX_ARM_PERMANENT_ID)
        self._install_failsafe_hooks()
        log.info("connected: %s", self.msp.identify())
        return self

    def _install_failsafe_hooks(self) -> None:
        """Send a final idle frame however this process ends.

        This board's MSP override has no timeout: whatever the FC last received
        is what it keeps applying, forever. So process death is not a safe
        state by default — it leaves the vehicle flying the last command.

        ``close()`` covers a graceful exit, and Python turns SIGINT into
        KeyboardInterrupt so ``finally`` blocks run. **SIGTERM is the gap**:
        Python installs no handler, so a plain ``kill`` terminates with zero
        cleanup. That is the likeliest way an operator or a service manager
        stops this process, which makes it the dangerous one.

        atexit covers normal exit and SystemExit; the SIGTERM handler covers
        ``kill`` and chains to whatever was installed before us rather than
        swallowing it.

        SIGKILL, power loss and USB unplug cannot be covered from software at
        all. That is exactly why the pilot's override switch and the pilot's
        throttle (under mask 11) are not optional.
        """
        if self._hooks_installed:
            return
        atexit.register(self._failsafe_idle)
        try:
            prev = signal.getsignal(signal.SIGTERM)

            def _on_sigterm(signum, frame):
                self._failsafe_idle()
                if callable(prev):
                    prev(signum, frame)
                else:
                    signal.signal(signal.SIGTERM, signal.SIG_DFL)
                    os.kill(os.getpid(), signum)

            signal.signal(signal.SIGTERM, _on_sigterm)
        except ValueError:
            # signal.signal() only works on the main thread; an FCLink built on
            # a worker still gets atexit coverage.
            log.debug("SIGTERM hook skipped (not main thread)")
        self._hooks_installed = True

    def _failsafe_idle(self) -> None:
        """Best-effort final idle frame. Safe to call repeatedly."""
        if self._closed or self.msp is None:
            return
        self._stop.set()
        try:
            # Explicitly DISARM in the last frame, not merely omit it. The
            # override has no timeout, so the FC keeps applying whatever it
            # last received — an omitted ARM channel leaves the previous HIGH
            # in force, and this frame is what a SIGTERM or an atexit unwind
            # leaves behind.
            arm = False if self.cfg.channels.companion_arm else None
            self.msp.request(MSP_SET_RAW_RC,
                             aetr_frame(**IDLE_STICKS, limits=self.limits,
                                        arm=arm),
                             timeout=0.3)
        except Exception:
            pass

    def close(self) -> None:
        if self._closed:
            return
        self._stop.set()
        for t in (self._rc_thread, self._telem_thread):
            if t is not None:
                t.join(timeout=1.0)
        if self.msp is not None:
            # Leave the FC holding a centred, throttle-floor frame. On this
            # board that is load-bearing, not tidiness: the override never
            # times out, so the last frame sent is what it keeps applying.
            self._failsafe_idle()
            self._closed = True
            self.msp.close()
        self._closed = True

    def __enter__(self):
        return self.connect() if self.msp is None else self

    def __exit__(self, *exc):
        self.close()

    # ----------------------------------------------------------- FC state

    def _status(self):
        p = self.msp.request(MSP_STATUS)
        import struct
        flags = struct.unpack("<I", p[6:10])[0]
        off = 13
        off += 2                # gyro cycle time
        off += 1 + p[off]       # extra flight-mode flag bytes
        off += 1                # arming-disable flag count
        adf = struct.unpack("<I", p[off:off + 4])[0]
        return flags, adf

    def armed(self) -> bool:
        return bool(self._status()[0] & (1 << self.arm_bit))

    def override_active(self) -> bool:
        if self.override_bit is None:
            return False
        return bool(self._status()[0] & (1 << self.override_bit))

    def rx_link_up(self) -> bool:
        return not (self._status()[1] & (1 << RXLOSS_BIT))

    def rc(self) -> list[int]:
        """All channels as the FC sees them: roll, pitch, yaw, throttle, aux..."""
        import struct
        p = self.msp.request(MSP_RC)
        return list(struct.unpack(f"<{len(p) // 2}H", p))

    def motors(self) -> list[int]:
        import struct
        p = self.msp.request(MSP_MOTOR)
        return list(struct.unpack(f"<{len(p) // 2}H", p))[:4]

    def attitude(self) -> dict:
        import struct
        r, p, y = struct.unpack("<hhh", self.msp.request(MSP_ATTITUDE)[:6])
        c = self.imu_cal
        return {"roll": c.roll_sign * r / 10.0,
                "pitch": c.pitch_sign * p / 10.0,
                "yaw": c.yaw_sign * float(y)}

    def attitude_q(self):
        """Current attitude as a body-to-world (FRD to NED) quaternion."""
        a = self.read_attitude() or self.attitude()
        return euler_to_q(a["roll"], a["pitch"], a["yaw"])

    def raw_imu(self):
        """(accel_frd_ms2, gyro_frd_rad_s) decoded per the IMU calibration."""
        import struct
        p = self.msp.request(MSP_RAW_IMU)
        vals = struct.unpack_from("<6h", p, 0)
        return decode_raw_imu(vals, self.imu_cal)

    def mode_ranges(self) -> list[dict]:
        """Aux switch bindings as the FC has them saved.

        Four bytes per entry: permanent box id, aux channel index (0 = AUX1 =
        ch5), start step, end step; each step is 25 us above 900. Entries with
        an empty range are unused slots.

        Read this rather than trusting the config's switch map — a board swap
        or a re-bind silently invalidates it, and a wrong override index means
        the operator believes a switch disengages the companion when it does
        not.
        """
        import struct
        p = self.msp.request(MSP_MODE_RANGES)
        names = {}
        try:
            ids = self.msp.box_ids()
            raw = self.msp.request(MSP_BOXNAMES).decode(errors="replace")
            labels = [s for s in raw.split(";") if s]
            names = dict(zip(ids, labels))
        except MSPError:
            pass
        out = []
        for i in range(0, len(p) - 3, 4):
            pid, aux, s0, s1 = struct.unpack_from("<4B", p, i)
            lo, hi = 900 + 25 * s0, 900 + 25 * s1
            if hi <= lo:
                continue          # unused slot
            out.append({"box_id": pid, "name": names.get(pid, f"id{pid}"),
                        "aux": aux + 1, "channel": aux + 5,
                        "index": aux + 4, "lo": lo, "hi": hi})
        return out

    def active_boxes(self) -> set:
        """Names of the flight-mode boxes currently active.

        Bit positions follow this build's box order, so the names are resolved
        from MSP_BOXIDS/MSP_BOXNAMES rather than hardcoded — they differ between
        firmware builds.
        """
        flags, _ = self._status()
        ids = self.msp.box_ids()
        raw = self.msp.request(MSP_BOXNAMES).decode(errors="replace")
        names = [s for s in raw.split(";") if s]
        return {n for i, n in enumerate(names) if i < len(ids) and flags & (1 << i)}

    def is_acro(self) -> bool:
        """True when no self-levelling mode is engaged.

        ACRO is the absence of ANGLE/HORIZON rather than a box of its own. It
        is what this stack is designed for: sticks command angular RATES, and a
        restrained airframe does not wind up the levelling I-term.
        """
        return not ({"ANGLE", "HORIZON"} & self.active_boxes())

    def state(self) -> dict:
        """One consistent snapshot, for logging and preflight printouts."""
        return {
            "armed": self.armed(),
            "override": self.override_active(),
            "rx_link": self.rx_link_up(),
            "rc": self.rc(),
            "motors": self.motors(),
            "attitude": self.attitude(),
            "abort": self.abort_reason,
            "crc_errors": self.msp.crc_errors,
        }

    # -------------------------------------------------------------- budget

    def txn_budget(self, rc_hz=None, telemetry_hz=None) -> dict:
        """Planned MSP transactions/sec against the FC's ceiling.

        Telemetry costs two transactions per cycle (attitude + raw IMU) and the
        watchdog costs two per check (status + motors). Over-subscribing does
        not fail loudly, it just makes every rate worse than configured, so the
        arithmetic is done up front where it can be seen.
        """
        rc = self.cfg.fc.rc_hz if rc_hz is None else rc_hz
        tel = self.cfg.fc.telemetry_hz if telemetry_hz is None else telemetry_hz
        # watchdog: status + motors, plus an MSP_RC readback when wire
        # verification is on.
        per_check = 3 if self.verify_wire else 2
        demand = rc + 2 * tel + per_check * self.cfg.fc.watchdog_hz
        ceiling = getattr(self.cfg.fc, "msp_txn_per_sec", MSP_TXN_PER_SEC)
        return {"rc": rc, "telemetry": tel, "watchdog": self.cfg.fc.watchdog_hz,
                "verify_wire": self.verify_wire,
                "demand": demand, "ceiling": ceiling,
                "load": demand / ceiling}

    def _check_budget(self, rc_hz=None, telemetry_hz=None) -> None:
        b = self.txn_budget(rc_hz, telemetry_hz)
        if b["load"] > MSP_BUDGET_WARN:
            log.warning(
                "MSP budget oversubscribed: %.0f txn/s demanded vs ~%.0f "
                "available (%.0f%%). RC %.0f Hz + telemetry %.0f Hz will each "
                "run slower than configured — lower them in config/vehicle.yaml",
                b["demand"], b["ceiling"], 100 * b["load"], b["rc"], b["telemetry"])

    # ------------------------------------------------------------- commands

    def set_stick(self, roll=None, pitch=None, yaw=None, throttle=None) -> None:
        """Update the commanded sticks (clamped when the frame is built).

        When the stream thread is running this only updates the setpoint; the
        thread keeps the override fresh. Without a stream thread it sends one
        frame immediately.
        """
        with self._slock:
            for name, val in (("roll", roll), ("pitch", pitch),
                              ("yaw", yaw), ("throttle", throttle)):
                if val is not None:
                    self._sticks[name] = val
            if throttle is not None:
                self._throttle_set_at = time.monotonic()
        if self._rc_thread is None or not self._rc_thread.is_alive():
            self.stream_once()

    def arm(self, on: bool = True) -> None:
        """Arm or disarm from the companion. OFF unless deliberately enabled.

        This is a real capability now, but it is not "self-arming": a human
        still initiates it, through software rather than the transmitter. The
        distinction that matters is whether a person decided, not which switch
        they touched.

        Refuses unless BOTH are true, because either alone is a foot-gun:

        * ``channels.companion_arm`` is set in the config. Arming is not
          something a caller should be able to reach by accident.
        * The pilot's override switch is ACTIVE. With it down the FC takes ARM
          from the receiver anyway, so arming here would silently do nothing
          and, worse, would leave a HIGH queued in our frame for whenever the
          switch next goes up.

        Disarming is always permitted and never refuses — a refusal on the way
        DOWN is the one failure this must not have.
        """
        if not on:
            with self._slock:
                self._arm_cmd = False
            log.warning("companion DISARM")
            return
        if not self.cfg.channels.companion_arm:
            raise PermissionError(
                "companion arming is disabled — ARM belongs to the pilot's "
                "transmitter. Set channels.companion_arm and put the ARM "
                "channel in msp_override_channels_mask to change that.")
        if not self.override_active():
            raise PermissionError(
                "refusing to arm while the override switch is DOWN: the FC "
                "would take ARM from the receiver regardless, and this would "
                "queue a HIGH for whenever the switch next goes up")
        if self._safe_mode or self.abort_reason:
            raise PermissionError(
                f"refusing to arm in safe mode: {self.abort_reason}")
        with self._slock:
            self._arm_cmd = True
        log.warning("companion ARM requested")

    def stream_once(self) -> None:
        """Send exactly one override frame.

        In safe mode this still sends — idle — because on this firmware
        silence means the FC holds the last command. See :meth:`_abort`.
        """
        with self._slock:
            sticks = dict(IDLE_STICKS) if self._safe_mode else dict(self._sticks)
            # Disarm is the RESTING state. Safe mode and any abort force it
            # low; nothing can leave it merely unset. See aetr_frame's `arm`.
            arm = (self._arm_cmd and not self._safe_mode) \
                if self.cfg.channels.companion_arm else None
        try:
            self.msp.request(MSP_SET_RAW_RC,
                             aetr_frame(**sticks, limits=self.limits, arm=arm),
                             timeout=0.2)
            self.frames_sent += 1
        except MSPError as e:
            self._abort(f"{ABORT_LINK_ERROR}: {e}")

    def start_rc_stream(self, hz: float | None = None) -> None:
        """Stream override frames until aborted or closed."""
        hz = hz or self.cfg.fc.rc_hz
        self._check_budget(rc_hz=hz)
        period = 1.0 / hz
        check_period = 1.0 / max(self.cfg.fc.watchdog_hz, 0.1)

        def loop():
            next_tick = time.perf_counter()
            next_check = 0.0
            while not self._stop.is_set():
                self.stream_once()
                now = time.perf_counter()
                # Once aborted, keep asserting idle but stop the readbacks —
                # they cost transactions and cannot change the outcome.
                if not self._safe_mode and now >= next_check:
                    next_check = now + check_period
                    self._watchdog()
                next_tick += period
                slp = next_tick - time.perf_counter()
                if slp > 0:
                    time.sleep(slp)
                else:
                    next_tick = time.perf_counter()

        self._rc_thread = threading.Thread(target=loop, name="rc-stream", daemon=True)
        self._rc_thread.start()

    def stop_rc_stream(self, reason: str = ABORT_REQUESTED) -> None:
        self._abort(reason)

    def _abort(self, reason: str) -> None:
        """Enter safe mode: force idle and KEEP STREAMING it.

        This deliberately does NOT stop the stream. Measured on this airframe
        (F405 / BTFL 4.5.1) 2026-08-04: the MSP override has **no timeout**.
        When frames stop, the FC keeps applying the last values it received,
        indefinitely — 5 s after the stream stopped it was still holding
        roll 1560 / throttle 1080, with the pilot's sticks ignored.

        So "abort by going silent" — which is what the earlier F722 / BTFL
        26.6.1 board did safely, handing back in ~250 ms — would here leave the
        vehicle flying the last command forever. Instead we keep the link alive
        and command centred sticks at the throttle floor, which is the safest
        thing we can still assert.

        The pilot's override switch remains the only cutout that works when
        this process cannot talk to the FC at all.
        """
        if self.abort_reason is None:
            self.abort_reason = reason
            log.warning("RC stream aborted -> commanding idle: %s", reason)
        self._safe_mode = True
        with self._slock:
            self._sticks.update(IDLE_STICKS)
            self._arm_cmd = False

    def _watchdog(self) -> None:
        """Periodic readbacks while streaming. Any doubt stops the stream.

        Disarm and override-release only count once the companion has actually
        had control: before that, streaming while disarmed is the normal way to
        keep frames fresh so the override can engage when the pilot flips up.
        """
        try:
            flags, _ = self._status()
            armed = bool(flags & (1 << self.arm_bit))
            override = (self.override_bit is not None
                        and bool(flags & (1 << self.override_bit)))
            if armed and override:
                self._engaged = True
            elif self._engaged:
                self._abort(ABORT_PILOT_DISARMED if not armed else ABORT_OVERRIDE_RELEASED)
                return
            if armed:
                mo = self.motors()
                if any(v > self.limits.motor_abort for v in mo):
                    self._abort(f"{ABORT_MOTOR_CAP}: {mo}")
            if override and self.verify_wire:
                self._verify_envelope()
            if override:
                self._check_throttle_freshness()
        except MSPError as e:
            self._abort(f"{ABORT_LINK_ERROR}: {e}")

    def _check_throttle_freshness(self) -> None:
        """Warn when throttle is being left to persist while override is active.

        set_stick() leaves an omitted channel at its previous value, and with
        msp_override_channels_mask = 15 the FC uses the companion's throttle,
        not the pilot's stick. So a module that steers attitude but never
        commands throttle freezes throttle at whatever was last sent, and the
        pilot's throttle stick cannot reduce it — their only remedy is the
        override switch. Legitimate constant throttle re-sends the same value
        every tick and never trips this.
        """
        # Under a mask that leaves ch3 to the pilot, the companion is SUPPOSED
        # not to command throttle; warning would be noise.
        if not (self.channels.override_mask & (1 << 2)):
            return
        if self._throttle_warned or self._throttle_set_at is None:
            return
        stale = time.monotonic() - self._throttle_set_at
        if stale > THROTTLE_STALE_S:
            self._throttle_warned = True
            log.warning(
                "throttle has not been commanded for %.1fs while the override "
                "is active — it is frozen at %d us and the pilot's throttle "
                "stick cannot change it. Either command throttle every tick, "
                "or set msp_override_channels_mask = 11 to leave ch3 with the "
                "pilot.", stale, self._sticks["throttle"])

    def _verify_envelope(self) -> None:
        """Confirm what the FC ACTUALLY received on ch1-4 is inside the envelope.

        ``aetr_frame`` clamps everything that goes through :meth:`set_stick`,
        but that is a cooperative guarantee: any code sharing this process — an
        in-process command module, a compiled extension, a stray debug script —
        can call ``msp.request(MSP_SET_RAW_RC, ...)`` directly and bypass it
        entirely. This checks the wire instead of trusting the caller.

        It is sampling, not prevention: a frame sent between two checks is
        acted on before it is seen. Treat it as a tripwire that stops the
        stream (returning control to the pilot), not as an interlock. Real
        prevention needs the module out of this process, or FC-side limits
        underneath it.

        Only meaningful while the override is active — with it off, MSP_RC
        carries the pilot's sticks, and the pilot is entitled to full range.
        """
        rc = self.rc()
        if len(rc) < 4:
            return
        # MSP_RC readback order is roll, pitch, yaw, throttle (throttle LAST),
        # unlike the AETR order we transmit. Verified on hardware 2026-08-04.
        roll, pitch, yaw, thr = rc[0], rc[1], rc[2], rc[3]
        tol = 2  # us, guards against off-by-one rounding rather than real drift
        lo = CENTER_US - self.limits.max_deflect - tol
        hi = CENTER_US + self.limits.max_deflect + tol
        mask = self.channels.override_mask
        bad = []
        # Only channels the FC is actually letting the companion drive. Under a
        # mask that leaves a channel to the pilot, MSP_RC carries the PILOT's
        # stick for it — and the pilot is entitled to the full range. Judging
        # that against the companion's envelope would abort the stream the
        # moment they advanced their own throttle.
        for name, v, bit in (("roll", roll, 0), ("pitch", pitch, 1), ("yaw", yaw, 3)):
            if (mask & (1 << bit)) and not lo <= v <= hi:
                bad.append(f"{name}={v} outside [{lo + tol}, {hi - tol}]")
        if (mask & (1 << 2)) and thr > self.limits.thr_cap + tol:
            bad.append(f"throttle={thr} above cap {self.limits.thr_cap}")
        # ARM, when the companion is driving it. The same bypass this whole
        # method exists to catch — anything in-process calling MSP_SET_RAW_RC
        # directly — could stream ARM HIGH, and that is the worst thing it
        # could do. So check the wire against our own intent rather than
        # trusting that nothing else is writing. An armed aircraft that this
        # code did not ask for is the abort, not a warning.
        ai = self.channels.arm_index
        if (self.channels.companion_arm and (mask & (1 << ai))
                and len(rc) > ai):
            with self._slock:
                wanted = self._arm_cmd
            fc_armed_cmd = rc[ai] >= self.channels.override_active_us
            if fc_armed_cmd and not wanted:
                bad.append(f"ARM={rc[ai]} on the wire but nothing asked to arm")
        if bad:
            self._abort(f"{ABORT_ENVELOPE}: " + "; ".join(bad)
                        + " — something bypassed set_stick()")

    # ------------------------------------------------------------ telemetry

    def start_telemetry(self, hz: float | None = None) -> None:
        hz = hz or self.cfg.fc.telemetry_hz
        self._check_budget(telemetry_hz=hz)
        period = 1.0 / hz

        def loop():
            while not self._stop.is_set():
                t0 = time.perf_counter()
                try:
                    att = self.attitude()
                    imu = self.raw_imu()
                except MSPError:
                    att = imu = None
                if att is not None:
                    with self._tlock:
                        self._att, self._imu = att, imu
                slp = period - (time.perf_counter() - t0)
                if slp > 0:
                    time.sleep(slp)

        self._telem_thread = threading.Thread(target=loop, name="telemetry", daemon=True)
        self._telem_thread.start()

    def read_attitude(self):
        """Cached attitude dict, or a fresh read when telemetry is not running."""
        if self._telem_thread is not None and self._telem_thread.is_alive():
            with self._tlock:
                return self._att
        return self.attitude()

    def read_imu(self):
        """Cached (accel_frd, gyro_frd), or a fresh read."""
        if self._telem_thread is not None and self._telem_thread.is_alive():
            with self._tlock:
                return self._imu
        return self.raw_imu()


def decode_raw_imu(vals, cal: IMUCal):
    """MSP_RAW_IMU 6 raw int16s -> (accel m/s^2, gyro rad/s) in body FRD.

    Applies the measured accelerometer scale, the gyro unit convention, and the
    board-to-FRD axis permutation. Everything downstream assumes FRD, so this
    is the single place the FC's sensor frame is reinterpreted.
    """
    ax, ay, az, gx, gy, gz = vals
    acc_board = np.array([ax, ay, az], float) * (G / cal.acc_per_g)
    gyro_scale = 1.0 if cal.gyro_units == "dps" else GYRO_LSB_2000DPS
    gyro_board = np.deg2rad(np.array([gx, gy, gz], float) * gyro_scale)
    return _to_frd(acc_board, cal), _to_frd(gyro_board, cal)


def _to_frd(v, cal: IMUCal):
    return np.array([sign * v[src] for src, sign in cal.board_to_frd], float)
