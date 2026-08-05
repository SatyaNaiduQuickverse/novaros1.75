"""The command envelope and channel ordering. No hardware needed."""

import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from companion.safety import (
    ARM_HIGH_US, ARM_LOW_US, AUX_NEUTRAL_US, IDX_ARM,  # noqa: E402
    IDX_PITCH, IDX_ROLL, IDX_THROTTLE, IDX_YAW, Limits, aetr_frame,
)


def unpack(frame):
    return struct.unpack("<4H", frame)


class TestAETROrdering(unittest.TestCase):
    def test_throttle_is_index_2(self):
        """The gotcha that once put 1550 on the throttle channel."""
        ch = unpack(aetr_frame(roll=1510, pitch=1520, yaw=1530, throttle=1050,
                               limits=Limits.bench()))
        self.assertEqual(ch[IDX_ROLL], 1510)
        self.assertEqual(ch[IDX_PITCH], 1520)
        self.assertEqual(ch[IDX_THROTTLE], 1050)
        self.assertEqual(ch[IDX_YAW], 1530)

    def test_frame_is_exactly_four_channels(self):
        """Aux channels must never be transmitted — ARM lives out there."""
        self.assertEqual(len(aetr_frame(1500, 1500, 1500, 1000, Limits.bench())), 8)


class TestClamps(unittest.TestCase):
    def setUp(self):
        self.lim = Limits.bench()

    def test_throttle_cannot_exceed_cap(self):
        for asked in (1101, 1500, 2000, 65535, float("inf")):
            ch = unpack(aetr_frame(1500, 1500, 1500, asked, self.lim))
            self.assertLessEqual(ch[IDX_THROTTLE], self.lim.thr_cap,
                                 f"throttle escaped the cap for {asked}")

    def test_throttle_cannot_go_below_floor(self):
        for asked in (999, 0, -500, float("-inf")):
            ch = unpack(aetr_frame(1500, 1500, 1500, asked, self.lim))
            self.assertGreaterEqual(ch[IDX_THROTTLE], self.lim.thr_floor)

    def test_garbage_degrades_to_a_safe_command(self):
        """A NaN out of a guidance solve must centre the sticks, not throw."""
        nan = float("nan")
        for bad in (nan, float("-inf"), None, "not-a-number"):
            ch = unpack(aetr_frame(bad, bad, bad, bad, self.lim))
            self.assertEqual(ch[IDX_THROTTLE], self.lim.thr_floor)
            for i in (IDX_ROLL, IDX_PITCH, IDX_YAW):
                self.assertEqual(ch[i], 1500)

    def test_deflection_is_bounded(self):
        lo, hi = 1500 - self.lim.max_deflect, 1500 + self.lim.max_deflect
        for asked in (0, 900, 2000, 5000):
            ch = unpack(aetr_frame(asked, asked, asked, 1000, self.lim))
            for i in (IDX_ROLL, IDX_PITCH, IDX_YAW):
                self.assertGreaterEqual(ch[i], lo)
                self.assertLessEqual(ch[i], hi)

    def test_every_output_is_a_valid_pwm_word(self):
        for asked in (-1e9, 0, 1500, 1e9):
            for ch in unpack(aetr_frame(asked, asked, asked, asked, self.lim)):
                self.assertTrue(900 <= ch <= 2100, f"{ch} is not a sane pulse width")

    def test_profiles_are_ordered(self):
        bench, teth, flight = (Limits.named(p) for p in ("bench", "tethered", "flight"))
        self.assertLess(bench.thr_cap, teth.thr_cap)
        self.assertLess(teth.thr_cap, flight.thr_cap)

    def test_unknown_profile_raises(self):
        with self.assertRaises(ValueError):
            Limits.named("yolo")


if __name__ == "__main__":
    unittest.main()


class TestArmChannelFraming(unittest.TestCase):
    """Packing ARM changes the frame length, and that IS the safety property.

    With `arm=None` the ARM channel is not in the frame at all, so it cannot be
    set by accident and the FC keeps taking ARM from the receiver. Only an
    explicit request lengthens the frame.
    """

    def test_default_frame_has_no_arm_channel(self):
        f = aetr_frame(1500, 1500, 1500, 1000, Limits.bench())
        self.assertEqual(len(f), 8, "default frame must stay 4 channels")

    def test_arm_false_packs_nine_channels_with_arm_low(self):
        f = aetr_frame(1500, 1500, 1500, 1000, Limits.bench(), arm=False)
        ch = struct.unpack("<9H", f)
        self.assertEqual(len(ch), 9)
        self.assertEqual(ch[IDX_ARM], ARM_LOW_US)

    def test_arm_true_packs_arm_high(self):
        f = aetr_frame(1500, 1500, 1500, 1000, Limits.bench(), arm=True)
        self.assertEqual(struct.unpack("<9H", f)[IDX_ARM], ARM_HIGH_US)

    def test_only_literal_true_arms(self):
        """A NaN, a string, a 1, a half-built object — all mean DISARMED.

        The only safe reading of an unclear intention on this channel is a
        refusal, so this is identity against True, not truthiness.
        """
        for sketchy in (1, "yes", [1], float("nan"), object()):
            f = aetr_frame(1500, 1500, 1500, 1000, Limits.bench(), arm=sketchy)
            self.assertEqual(struct.unpack("<9H", f)[IDX_ARM], ARM_LOW_US,
                             f"{sketchy!r} must not arm")

    def test_arming_does_not_relax_the_stick_clamps(self):
        f = aetr_frame(9999, -9999, 9999, 9999, Limits.bench(), arm=True)
        ch = struct.unpack("<9H", f)
        self.assertEqual(ch[IDX_THROTTLE], Limits.bench().thr_cap)
        self.assertEqual(ch[IDX_ROLL], 1500 + Limits.bench().max_deflect)

    def test_padding_channels_are_neutral_not_stale(self):
        """ch5-8 are padded, so an enabled mask bit there cannot pick up junk."""
        ch = struct.unpack("<9H", aetr_frame(1500, 1500, 1500, 1000,
                                             Limits.bench(), arm=False))
        for i in range(4, IDX_ARM):
            self.assertEqual(ch[i], AUX_NEUTRAL_US)
