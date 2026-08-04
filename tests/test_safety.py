"""The command envelope and channel ordering. No hardware needed."""

import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from companion.safety import (  # noqa: E402
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
