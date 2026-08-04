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
PINNED_SHA = "f52b7f8f91c34c66a78cea12be1babdd2c426d8ca51238bb083ef274ddfad1cd"


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

    def test_thrust_compensation_fingerprint(self):
        """Computed throttle rises with tilt — and exceeds our 1100 bench cap."""
        fc, imu, g = self.build()
        for tilt, expect in ((0, 1224), (30, 1252), (45, 1298)):
            imu.q = q_roll(float(tilt))
            self.assertEqual(g.step(0.0)["throttle"], expect)

    def test_computed_throttle_exceeds_bench_cap(self):
        """Documents WHY the returned dict must never be applied raw."""
        from companion.safety import Limits
        fc, imu, g = self.build()
        imu.q = q_roll(0.0)
        self.assertGreater(g.step(0.0)["throttle"], Limits.bench().thr_cap)

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
