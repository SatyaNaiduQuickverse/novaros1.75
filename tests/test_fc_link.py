"""FCLink control-authority behaviour, against a fake FC. No hardware needed.

These lock down the rules that keep a pilot in charge: the companion never
arms, never sends aux channels, and stops streaming the moment anything is off.
"""

import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from companion.config import Config, load  # noqa: E402
from companion.fc_link import FCLink  # noqa: E402
from companion.msp import (  # noqa: E402
    MSPError, MSP_MOTOR, MSP_RC, MSP_SET_RAW_RC, MSP_STATUS,
)
from companion.safety import (
    ARM_LOW_US, IDX_ARM,  # noqa: E402
    ABORT_MOTOR_CAP, ABORT_OVERRIDE_RELEASED, ABORT_PILOT_DISARMED,
)

ARM_BIT, OVERRIDE_BIT = 0, 26


def status_payload(flags, arming_disable=0):
    return (struct.pack("<HHH", 500, 0, 0)          # cycle, i2c errs, sensors
            + struct.pack("<I", flags)              # mode flags        [6:10]
            + bytes([0])                            # profile           [10]
            + struct.pack("<H", 10)                 # system load       [11:13]
            + struct.pack("<H", 125)                # gyro cycle time   [13:15]
            + bytes([0])                            # extra mode bytes  [15]
            + bytes([0])                            # arming flag count [16]
            + struct.pack("<I", arming_disable))    # armingDisable     [17:21]


class FakeMSP:
    """Records outgoing RC frames and answers state queries from attributes."""

    def __init__(self):
        self.armed = False
        self.override = False
        self.motor = 1050
        self.rc_frames = []
        self.crc_errors = 0
        self.fail_next = False
        # What MSP_RC reports back: roll, pitch, yaw, throttle, then aux.
        self.rc_readback = [1500, 1500, 1500, 1000] + [1000] * 5

    def request(self, cmd, payload=b"", timeout=None):
        if self.fail_next:
            raise MSPError("simulated link loss")
        if cmd == MSP_SET_RAW_RC:
            self.rc_frames.append(struct.unpack(f"<{len(payload) // 2}H", payload))
            return b""
        if cmd == MSP_STATUS:
            flags = (self.armed << ARM_BIT) | (self.override << OVERRIDE_BIT)
            return status_payload(flags)
        if cmd == MSP_MOTOR:
            return struct.pack("<4H", *([self.motor] * 4))
        if cmd == MSP_RC:
            return struct.pack(f"<{len(self.rc_readback)}H", *self.rc_readback)
        raise MSPError(f"unexpected cmd {cmd}")

    def close(self):
        pass


def make_link():
    fc = FCLink(Config())
    fc.msp = FakeMSP()
    fc.arm_bit, fc.override_bit = ARM_BIT, OVERRIDE_BIT
    return fc


class TestPilotAuthority(unittest.TestCase):
    def test_companion_cannot_arm(self):
        with self.assertRaises(PermissionError):
            make_link().arm(True)

    def test_only_four_channels_are_ever_sent(self):
        fc = make_link()
        fc.set_stick(roll=1600, pitch=1400, yaw=1550, throttle=1090)
        self.assertEqual(len(fc.msp.rc_frames[-1]), 4,
                         "aux channels must never be transmitted")

    def test_commands_are_clamped_on_the_wire(self):
        fc = make_link()
        fc.set_stick(roll=2000, throttle=2000)
        roll, _, thr, _ = fc.msp.rc_frames[-1]
        self.assertEqual(thr, fc.limits.thr_cap)
        self.assertEqual(roll, 1500 + fc.limits.max_deflect)


class TestWireVerification(unittest.TestCase):
    """The aetr_frame() clamps are cooperative — anything sharing this process
    can call msp.request(MSP_SET_RAW_RC, ...) directly and bypass them. The
    watchdog reads back what the FC actually got and stops the stream if it is
    outside the envelope. Detection, not prevention.
    """

    def engaged(self):
        fc = make_link()
        fc.msp.armed = fc.msp.override = True
        return fc

    def test_in_envelope_readback_is_fine(self):
        fc = self.engaged()
        fc.msp.rc_readback[:4] = [1560, 1440, 1530, 1050]
        fc._watchdog()
        self.assertIsNone(fc.abort_reason)

    def test_throttle_above_cap_on_the_wire_aborts(self):
        fc = self.engaged()
        fc.msp.rc_readback[:4] = [1500, 1500, 1500, 1400]
        fc._watchdog()
        self.assertIn("throttle=1400", fc.abort_reason)
        self.assertIn("bypassed set_stick", fc.abort_reason)

    def test_excess_deflection_on_the_wire_aborts(self):
        fc = self.engaged()
        fc.msp.rc_readback[:4] = [1900, 1500, 1500, 1000]
        fc._watchdog()
        self.assertIn("roll=1900", fc.abort_reason)

    def test_readback_uses_rpyt_order_not_aetr(self):
        """MSP_RC returns throttle LAST; reading it as AETR would mis-flag."""
        fc = self.engaged()
        fc.msp.rc_readback[:4] = [1500, 1500, 1000, 1100]   # yaw 1000, thr 1100
        fc._watchdog()
        self.assertIsNotNone(fc.abort_reason)   # yaw 1000 is outside ±100
        self.assertIn("yaw=1000", fc.abort_reason)

    def test_skipped_when_the_pilot_has_control(self):
        """With override off MSP_RC carries the pilot's sticks, and the pilot
        is entitled to the full range."""
        fc = make_link()
        fc.msp.rc_readback[:4] = [2000, 1000, 2000, 1900]
        fc._watchdog()
        self.assertIsNone(fc.abort_reason)

    def test_can_be_disabled(self):
        fc = self.engaged()
        fc.verify_wire = False
        fc.msp.rc_readback[:4] = [1900, 1500, 1500, 1900]
        fc._watchdog()
        self.assertIsNone(fc.abort_reason)


class TestMSPBudget(unittest.TestCase):
    """The FC services MSP at ~99 transactions/s, shared by every thread.

    Over-subscribing degrades all rates silently rather than erroring, so the
    arithmetic is checked up front. Measured on hardware 2026-08-04:
    rc_hz 100 + telemetry_hz 25 actually delivered 46 Hz RC.
    """

    def test_telemetry_costs_two_transactions_per_cycle(self):
        fc = make_link()
        fc.cfg.fc.watchdog_hz = 0
        b = fc.txn_budget(rc_hz=50, telemetry_hz=10)
        self.assertEqual(b["demand"], 70)

    def test_watchdog_is_counted(self):
        fc = make_link()
        fc.cfg.fc.watchdog_hz = 4
        fc.verify_wire = False
        self.assertEqual(fc.txn_budget(rc_hz=50, telemetry_hz=0)["demand"], 58)

    def test_wire_verification_costs_one_more_per_check(self):
        fc = make_link()
        fc.cfg.fc.watchdog_hz = 4
        fc.verify_wire = False
        without = fc.txn_budget(rc_hz=50, telemetry_hz=0)["demand"]
        fc.verify_wire = True
        self.assertEqual(fc.txn_budget(rc_hz=50, telemetry_hz=0)["demand"],
                         without + 4)

    def test_shipped_config_fits_under_the_ceiling(self):
        import os
        from companion.config import load
        repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        cfg = load(os.path.join(repo, "config", "vehicle.yaml"))
        fc = FCLink(cfg)
        self.assertLessEqual(fc.txn_budget()["load"], 1.0,
                             "shipped rates demand more than the FC can serve")

    def test_oversubscription_warns(self):
        fc = make_link()
        with self.assertLogs("companion.fc", level="WARNING") as cm:
            fc._check_budget(rc_hz=100, telemetry_hz=25)
        self.assertIn("oversubscribed", cm.output[0])

    def test_sane_rates_do_not_warn(self):
        fc = make_link()
        fc.cfg.fc.watchdog_hz = 2
        with self.assertNoLogs("companion.fc", level="WARNING"):
            fc._check_budget(rc_hz=30, telemetry_hz=5)

    def test_ceiling_is_per_board(self):
        """F722/BTFL26.6.1 served ~99 txn/s; F405/BTFL4.5.1 serves ~62. Rates
        that were fine on one board oversubscribe the other."""
        fc = make_link()
        fc.cfg.fc.watchdog_hz = 3
        fc.cfg.fc.msp_txn_per_sec = 99.0
        self.assertLess(fc.txn_budget(rc_hz=50, telemetry_hz=12)["load"], 0.9)
        fc.cfg.fc.msp_txn_per_sec = 62.0
        self.assertGreater(fc.txn_budget(rc_hz=50, telemetry_hz=12)["load"], 1.0)


class TestWatchdog(unittest.TestCase):
    def test_streaming_while_disarmed_is_allowed(self):
        """Frames must keep flowing before takeover, so the override can engage."""
        fc = make_link()
        fc._watchdog()
        self.assertIsNone(fc.abort_reason)

    def test_disarm_after_engagement_aborts(self):
        fc = make_link()
        fc.msp.armed = fc.msp.override = True
        fc._watchdog()
        self.assertTrue(fc._engaged)
        fc.msp.armed = False
        fc._watchdog()
        self.assertEqual(fc.abort_reason, ABORT_PILOT_DISARMED)

    def test_pilot_retaking_override_aborts(self):
        fc = make_link()
        fc.msp.armed = fc.msp.override = True
        fc._watchdog()
        fc.msp.override = False
        fc._watchdog()
        self.assertEqual(fc.abort_reason, ABORT_OVERRIDE_RELEASED)

    def test_motor_above_cap_aborts(self):
        fc = make_link()
        fc.msp.armed = fc.msp.override = True
        fc.msp.motor = fc.limits.motor_abort + 1
        fc._watchdog()
        self.assertIn(ABORT_MOTOR_CAP, fc.abort_reason)

    def test_link_error_aborts(self):
        fc = make_link()
        fc.msp.fail_next = True
        fc._watchdog()
        self.assertIn("serial/MSP error", fc.abort_reason)

    def test_abort_keeps_streaming_idle(self):
        """Abort must ASSERT idle, not go silent.

        Measured on the F405 / BTFL 4.5.1 board 2026-08-04: the MSP override
        has no timeout. When frames stop, the FC keeps applying the last values
        it received indefinitely — 5 s after the stream stopped it still held
        roll 1560 / throttle 1080 with the pilot's sticks ignored. So going
        silent would leave the vehicle flying the last command. The earlier
        F722 / BTFL 26.6.1 board did fall back in ~250 ms; this behaviour is
        firmware-dependent and must not be assumed.
        """
        fc = make_link()
        fc.set_stick(roll=1580, pitch=1420, yaw=1560, throttle=1090)
        fc.stop_rc_stream()
        before = len(fc.msp.rc_frames)
        fc.stream_once()
        self.assertEqual(len(fc.msp.rc_frames), before + 1,
                         "abort must keep sending, not go silent")
        roll, pitch, thr, yaw = fc.msp.rc_frames[-1]     # wire order is AETR
        self.assertEqual((roll, pitch, yaw), (1500, 1500, 1500))
        self.assertEqual(thr, 1000)

    def test_safe_mode_ignores_further_commands(self):
        """Once aborted, a command module cannot re-assert control."""
        fc = make_link()
        fc.stop_rc_stream()
        fc.set_stick(roll=1600, throttle=1100)
        fc.stream_once()
        roll, _, thr, _ = fc.msp.rc_frames[-1]
        self.assertEqual((roll, thr), (1500, 1000))

    def test_first_abort_reason_is_kept(self):
        fc = make_link()
        fc.stop_rc_stream("first")
        fc.stop_rc_stream("second")
        self.assertEqual(fc.abort_reason, "first")


if __name__ == "__main__":
    unittest.main()


class TestEngagementEpoch(unittest.TestCase):
    """t is seconds since engage(). A phased module handed any other epoch
    starts in a late phase silently, so the reference placeholder warns.

    The currently-deployed compiled module is attitude-only and ignores t
    entirely (verified: identical output at t = 0, 5, 30, 60 s), so the guard
    is moot for it — but it must keep working for any future module that does
    sequence off t.
    """

    def build(self):
        # Load the REFERENCE PLACEHOLDER explicitly by path. Once a compiled
        # module is deployed, `companion.command_module` resolves to the .so
        # (extensions take precedence, and the placeholder is renamed aside),
        # so importing by name here would silently test the delivered binary
        # instead of the contract guard these tests exist to cover.
        import importlib.util, os
        repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        ph = os.path.join(repo, "companion", "command_module.py.placeholder")
        if not os.path.exists(ph):
            ph = os.path.join(repo, "companion", "command_module.py")
        # The loader must be explicit: importlib infers it from the file
        # extension, and ".py.placeholder" is not one it recognises.
        # Must load INSIDE the companion package: the placeholder uses
        # relative imports (`from .safety import ...`), which only resolve if
        # the module name is package-qualified. The loader must also be
        # explicit, since ".py.placeholder" is not an extension importlib
        # recognises.
        from importlib.machinery import SourceFileLoader
        import companion  # noqa: F401  (ensures the package is initialised)
        name = "companion._placeholder_cm"
        spec = importlib.util.spec_from_file_location(
            name, ph, loader=SourceFileLoader(name, ph))
        mod = importlib.util.module_from_spec(spec)
        sys.modules[name] = mod
        spec.loader.exec_module(mod)
        CommandModule = mod.CommandModule
        fc = make_link()

        class _IMU:
            def get_state(self):
                import numpy as np
                return (np.array([1.0, 0, 0, 0]), np.zeros(3), np.zeros(3))

        class _Vis:
            def bearing(self, t):
                return None

        return CommandModule(fc, _IMU(), _Vis())

    def test_engage_starts_the_clock(self):
        g = self.build()
        self.assertFalse(g.engaged)
        g.engage()
        self.assertTrue(g.engaged)
        self.assertEqual(g.ticks, 0)

    def test_near_zero_first_tick_is_accepted(self):
        g = self.build()
        g.engage()
        with self.assertNoLogs("companion.command", level="WARNING"):
            g.step(0.0)
            g.step(0.02)

    def test_absolute_clock_on_first_tick_warns(self):
        """time.monotonic() is uptime — large, so the first tick is not ~0."""
        g = self.build()
        g.engage()
        with self.assertLogs("companion.command", level="WARNING") as cm:
            g.step(48213.7)
        self.assertIn("seconds since engage", cm.output[0])

    def test_time_going_backwards_warns(self):
        g = self.build()
        g.engage()
        g.step(0.0)
        g.step(1.0)
        with self.assertLogs("companion.command", level="WARNING"):
            g.step(0.5)

    def test_engage_resets_tick_count(self):
        g = self.build()
        g.engage()
        g.step(0.0)
        g.step(0.1)
        self.assertEqual(g.ticks, 2)
        g.engage()
        self.assertEqual(g.ticks, 0)


class TestOverrideMask(unittest.TestCase):
    """The wire-envelope check must police only channels the companion drives.

    With msp_override_channels_mask = 11 the pilot keeps throttle, and MSP_RC
    carries THEIR stick for it. Judging that against the companion's cap would
    abort the stream the instant they advanced their own throttle — during an
    armed bench test, that is exactly the wrong moment.
    """

    def engaged(self, mask):
        fc = make_link()
        fc.msp.armed = fc.msp.override = True
        fc.channels.override_mask = mask
        return fc

    def test_mask15_polices_throttle(self):
        fc = self.engaged(15)
        fc.msp.rc_readback[:4] = [1500, 1500, 1500, 1600]
        fc._watchdog()
        self.assertIn("throttle=1600", fc.abort_reason)

    def test_mask11_ignores_pilot_throttle(self):
        fc = self.engaged(11)
        fc.msp.rc_readback[:4] = [1500, 1500, 1500, 1900]   # pilot's own stick
        fc._watchdog()
        self.assertIsNone(fc.abort_reason,
                          "false abort on the pilot's legitimate throttle")

    def test_mask11_still_polices_attitude(self):
        fc = self.engaged(11)
        fc.msp.rc_readback[:4] = [1900, 1500, 1500, 1900]
        fc._watchdog()
        self.assertIn("roll=1900", fc.abort_reason)

    def test_mask11_silences_throttle_freshness_warning(self):
        fc = self.engaged(11)
        fc._throttle_set_at = 0.0            # ancient
        with self.assertNoLogs("companion.fc", level="WARNING"):
            fc._check_throttle_freshness()

    def test_mask15_warns_on_stale_throttle(self):
        fc = self.engaged(15)
        fc._throttle_set_at = 0.0
        with self.assertLogs("companion.fc", level="WARNING") as cm:
            fc._check_throttle_freshness()
        self.assertIn("frozen", cm.output[0])


class TestFailsafeHooks(unittest.TestCase):
    """Process death must leave the FC holding idle, not the last command.

    This board's MSP override never times out, so whatever the FC last received
    is what it keeps applying. Hardware-measured 2026-08-04 with override up and
    disarmed, victim streaming [1580,1420,1560,1090]:

        SIGINT  (Ctrl-C)  -> FC holds [1500,1500,1500,1000]  safe
        SIGTERM (kill)    -> FC holds [1500,1500,1500,1000]  safe (needs the hook)
        SIGKILL (kill -9) -> FC holds [1580,1420,1560,1090]  UNFIXABLE

    SIGTERM is the one that needs code: Python installs no handler, so a plain
    kill terminates with zero cleanup. It is also the likeliest way an operator
    or a service manager stops this process.
    """

    def test_hooks_are_installed_on_connect(self):
        fc = make_link()
        fc._install_failsafe_hooks()
        self.assertTrue(fc._hooks_installed)

    def test_sigterm_handler_is_registered(self):
        import signal
        fc = make_link()
        before = signal.getsignal(signal.SIGTERM)
        try:
            fc._install_failsafe_hooks()
            self.assertIsNot(signal.getsignal(signal.SIGTERM), before,
                             "SIGTERM must be hooked; a plain kill otherwise "
                             "leaves the FC flying the last command")
        finally:
            signal.signal(signal.SIGTERM, before)

    def test_failsafe_idle_sends_idle(self):
        fc = make_link()
        fc.set_stick(roll=1580, pitch=1420, yaw=1560, throttle=1090)
        fc._failsafe_idle()
        roll, pitch, thr, yaw = fc.msp.rc_frames[-1]
        self.assertEqual((roll, pitch, yaw, thr), (1500, 1500, 1500, 1000))

    def test_failsafe_idle_is_idempotent_and_never_raises(self):
        fc = make_link()
        fc._failsafe_idle()
        fc.msp.fail_next = True
        fc._failsafe_idle()          # must not raise during teardown
        fc._closed = True
        fc._failsafe_idle()

    def test_close_is_idempotent(self):
        fc = make_link()
        fc.close()
        fc.close()
        self.assertTrue(fc._closed)


class TestCompanionArming(unittest.TestCase):
    """Arming from code — a human still decides, via a button not a switch.

    The hazard this is built around is specific to this board: the MSP override
    has NO timeout, so whatever is streamed on the ARM channel is what the FC
    keeps applying, forever. Disarm therefore has to be the RESTING state — the
    value sent when nothing has asked for anything — rather than an event the
    code has to remember. Every test below is a variation on that.
    """

    def _fc(self, companion_arm=True, override=True, mask=271):
        cfg = load()
        # Explicit mask: arming needs the ARM channel in it, and the repo
        # config's value moves with whatever the airframe is set to today.
        cfg.channels.override_mask = mask
        cfg.channels.companion_arm = companion_arm
        fc = FCLink.__new__(FCLink)
        FCLink.__init__(fc, cfg)
        fc.msp = None
        fc._arm_cmd = False
        fc._safe_mode = False
        fc.abort_reason = None
        fc.override_active = lambda: override
        return fc

    def test_disabled_by_default_in_the_dataclass(self):
        from companion.config import ChannelConfig
        self.assertFalse(ChannelConfig().companion_arm)

    def test_arming_refuses_when_the_capability_is_off(self):
        fc = self._fc(companion_arm=False)
        with self.assertRaises(PermissionError):
            fc.arm(True)
        self.assertFalse(fc._arm_cmd)

    def test_arming_refuses_while_the_override_switch_is_down(self):
        """The FC would take ARM from the receiver anyway — and this would
        queue a HIGH for whenever the switch next goes up."""
        fc = self._fc(override=False)
        with self.assertRaises(PermissionError):
            fc.arm(True)
        self.assertFalse(fc._arm_cmd)

    def test_arming_refuses_in_safe_mode(self):
        fc = self._fc()
        fc._safe_mode = True
        fc.abort_reason = "something already went wrong"
        with self.assertRaises(PermissionError):
            fc.arm(True)

    def test_disarming_never_refuses(self):
        """A refusal on the way DOWN is the one failure this must not have."""
        for kw in ({"companion_arm": False}, {"override": False}):
            fc = self._fc(**kw)
            fc._arm_cmd = True
            fc.arm(False)                      # must not raise
            self.assertFalse(fc._arm_cmd)

    def test_abort_drops_the_arm_request(self):
        fc = self._fc()
        fc._arm_cmd = True
        fc._abort("link died")
        self.assertFalse(fc._arm_cmd)

    def test_arm_survives_nothing_it_should_not(self):
        fc = self._fc()
        fc.arm(True)
        self.assertTrue(fc._arm_cmd)
        fc._abort("pilot dropped the switch")
        self.assertFalse(fc._arm_cmd, "an abort must disarm, not just idle")


class TestArmFrameOnTheWire(unittest.TestCase):
    """What actually reaches the FC, including on the paths nobody calls."""

    def _spy(self, companion_arm=True):
        cfg = load()
        cfg.channels.companion_arm = companion_arm
        fc = FCLink.__new__(FCLink)
        FCLink.__init__(fc, cfg)
        sent = []

        class Spy:
            def request(self_inner, cmd, payload=b"", timeout=None):
                sent.append((cmd, payload))
                return b""
        fc.msp = Spy()
        fc._arm_cmd = False
        fc._safe_mode = False
        fc.abort_reason = None
        fc._closed = False
        return fc, sent

    @staticmethod
    def _arm_of(payload):
        n = len(payload) // 2
        return struct.unpack(f"<{n}H", payload)[IDX_ARM] if n > IDX_ARM else None

    def test_capability_off_never_lengthens_the_frame(self):
        fc, sent = self._spy(companion_arm=False)
        fc.stream_once()
        self.assertEqual(len(sent[0][1]), 8)

    def test_resting_state_streams_arm_low_every_frame(self):
        fc, sent = self._spy()
        fc.stream_once()
        self.assertEqual(self._arm_of(sent[0][1]), ARM_LOW_US)

    def test_safe_mode_forces_arm_low_even_if_requested(self):
        """Belt and braces: _abort clears the flag, and this ignores it anyway."""
        fc, sent = self._spy()
        fc._arm_cmd = True
        fc._safe_mode = True
        fc.stream_once()
        self.assertEqual(self._arm_of(sent[0][1]), ARM_LOW_US)

    def test_the_final_failsafe_frame_disarms_explicitly(self):
        """This is the frame a SIGTERM or an atexit unwind leaves behind.

        It must SAY disarmed, not merely omit ARM — an omitted channel leaves
        the FC applying the previous HIGH, forever, on this firmware.
        """
        fc, sent = self._spy()
        fc._arm_cmd = True
        fc._failsafe_idle()
        self.assertEqual(self._arm_of(sent[-1][1]), ARM_LOW_US)


class TestArmTripwire(unittest.TestCase):
    """The wire check must police ARM once the companion can drive it.

    Everything in safety.py is cooperative — anything sharing this process can
    call MSP_SET_RAW_RC directly and bypass every clamp. Streaming ARM HIGH is
    the worst thing such a bypass could do, so the watchdog compares the wire
    against our own intent rather than trusting that nothing else is writing.
    """

    def _fc(self, rc, arm_cmd=False, mask=271, companion_arm=True):
        cfg = load()
        cfg.channels.override_mask = mask
        cfg.channels.companion_arm = companion_arm
        fc = FCLink.__new__(FCLink)
        FCLink.__init__(fc, cfg)
        fc.msp = None
        fc._arm_cmd = arm_cmd
        fc._safe_mode = False
        fc.abort_reason = None
        fc.rc = lambda: rc
        return fc

    @staticmethod
    def _rc(arm_us):
        return [1500, 1500, 1500, 1000, 1000, 1000, 1000, 2000, arm_us]

    def test_aborts_when_the_wire_is_armed_and_nothing_asked(self):
        fc = self._fc(self._rc(2000), arm_cmd=False)
        fc._verify_envelope()
        self.assertIsNotNone(fc.abort_reason)
        self.assertIn("ARM", fc.abort_reason)

    def test_quiet_when_the_wire_matches_our_intent(self):
        fc = self._fc(self._rc(2000), arm_cmd=True)
        fc._verify_envelope()
        self.assertIsNone(fc.abort_reason)

    def test_quiet_when_disarmed_on_both_sides(self):
        fc = self._fc(self._rc(1000), arm_cmd=False)
        fc._verify_envelope()
        self.assertIsNone(fc.abort_reason)

    def test_silent_when_arm_is_not_in_the_mask(self):
        """Mask 15: ch9 carries the PILOT's switch, and they may arm freely."""
        fc = self._fc(self._rc(2000), arm_cmd=False, mask=15)
        fc._verify_envelope()
        self.assertIsNone(fc.abort_reason)

    def test_silent_when_the_capability_is_off(self):
        fc = self._fc(self._rc(2000), arm_cmd=False, companion_arm=False)
        fc._verify_envelope()
        self.assertIsNone(fc.abort_reason)
