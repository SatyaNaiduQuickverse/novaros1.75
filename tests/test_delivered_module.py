"""Behavioural fingerprint of the delivered compiled command module.

Identity of this binary rests on behaviour, not on an externally-certified
digest: the supplier builds on x86 and could not produce an independent
aarch64 digest, so the pinned SHA originates from the delivered file itself
and is change-detection only. These numbers were measured here BEFORE the
supplier quoted them, and matching them is what established that the deployed
binary implements the certified control law.

Skips cleanly when the .so is absent, so the suite still runs elsewhere.
"""

import importlib.util
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SO = os.path.join(REPO, "companion",
                  "command_module.cpython-313-aarch64-linux-gnu.so")
PINNED_SHA = "bf6d045b7976c3ea8abed1fcaa1f06567cf8b70ad6238c4f5f42957048e07f4c"


def q_roll(deg):
    a = math.radians(deg) / 2.0
    return np.array([math.cos(a), math.sin(a), 0.0, 0.0])


class SpyFC:
    def __init__(self):
        self.sent = []
        self.arm_attempts = 0

    def set_stick(self, roll=None, pitch=None, yaw=None, throttle=None):
        self.sent.append({k: v for k, v in
                          (("roll", roll), ("pitch", pitch),
                           ("yaw", yaw), ("throttle", throttle)) if v is not None})

    def arm(self, on=True):
        self.arm_attempts += 1


class SpyIMU:
    def __init__(self):
        self.q = q_roll(0.0)

    def get_state(self):
        return self.q, np.zeros(3), np.array([0.0, 0.0, -9.81])


@unittest.skipUnless(os.path.exists(SO), "compiled command module not present")
class TestDeliveredModule(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location("command_module", SO)
        cls.mod = importlib.util.module_from_spec(spec)
        sys.modules["command_module"] = cls.mod
        spec.loader.exec_module(cls.mod)

    def build(self):
        fc, imu = SpyFC(), SpyIMU()
        return fc, imu, self.mod.CommandModule(fc, imu, None, None)

    def test_pinned_digest(self):
        import hashlib
        h = hashlib.sha256()
        with open(SO, "rb") as f:
            for chunk in iter(lambda: f.read(1 << 20), b""):
                h.update(chunk)
        self.assertEqual(h.hexdigest(), PINNED_SHA,
                         "delivered binary changed — re-run the delivery check")

    def test_roll_correction_opposes_the_tilt(self):
        """The sign that matters. Inverted, this becomes positive feedback."""
        fc, imu, g = self.build()
        imu.q = q_roll(+15.0)
        self.assertEqual(g.step(0.0)["roll"], 1439)
        imu.q = q_roll(-15.0)
        self.assertEqual(g.step(0.02)["roll"], 1560)

    def test_emits_no_throttle_at_any_tilt(self):
        """The delivered build is attitude-only — throttle is not in the dict.

        The previous build computed a thrust-vector throttle (1224 level, 1298
        at 45 deg) and exposed it in the returned dict. That was ABOVE the 1100
        bench cap, so the standing rule was that the dict must never be applied
        raw. This build removes the question: under mask 11 the pilot owns ch3,
        and the module does not compute a throttle at all.

        Asserted across tilt because "absent at zero" would not be the claim
        that matters — the old build's throttle Rose with tilt.
        """
        fc, imu, g = self.build()
        for tilt in (0, 15, 30, 45, -30):
            imu.q = q_roll(float(tilt))
            out = g.step(0.0)
            self.assertNotIn("throttle", out, f"throttle appeared at {tilt} deg")

    def test_never_touches_the_throttle_channel_on_the_fc(self):
        """Not just absent from the dict — never passed to set_stick either."""
        fc, imu, g = self.build()
        for tilt in (0, 20, -20, 40):
            imu.q = q_roll(float(tilt))
            g.step(0.0)
        for frame in fc.sent:
            self.assertNotIn("throttle", frame,
                             "the module must leave ch3 to the pilot")
        self.assertTrue(fc.sent, "the module commanded nothing at all")

    def test_never_sends_throttle(self):
        fc, imu, g = self.build()
        for tilt in (0, 10, 30, 45):
            imu.q = q_roll(float(tilt))
            g.step(0.0)
        self.assertTrue(fc.sent)
        self.assertFalse(any("throttle" in s for s in fc.sent),
                         "pilot owns ch3 under mask 11")

    def test_never_arms(self):
        fc, imu, g = self.build()
        g.step(0.0)
        self.assertEqual(fc.arm_attempts, 0)

    def test_attitude_only_no_time_dependence(self):
        """No engage(), no phases — output must depend on attitude alone."""
        fc, imu, g = self.build()
        imu.q = q_roll(0.0)
        first = g.step(0.0)
        for t in (5.0, 30.0, 60.0):
            self.assertEqual(g.step(t), first)
