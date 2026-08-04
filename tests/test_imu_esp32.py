"""ESP32/MPU6500 bridge: frame parsing, scaling, and the attitude filter.

No hardware — frames are synthesised, so the wire format and the filter can
regress independently of whether a board is plugged in.
"""

import math
import time
import os
import struct
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from companion.imu_esp32 import (  # noqa: E402
    ACCEL_LSB_PER_G, FRAME_LEN, GYRO_LSB_PER_DPS, SYNC, ESP32IMU, Mahony,
)
from companion.math_utils import q_to_R, q_to_euler  # noqa: E402

G = 9.80665


def frame(seq, ax, ay, az, gx, gy, gz, temp=0, corrupt=False):
    body = struct.pack("<B7h", seq & 0xFF, ax, ay, az, temp, gx, gy, gz)
    x = 0
    for b in body:
        x ^= b
    if corrupt:
        x ^= 0xFF
    return SYNC + body + bytes([x])


class _Cal:
    gyro_bias = (0.0, 0.0, 0.0)
    accel_per_g = ACCEL_LSB_PER_G
    axis_map = ((0, 1.0), (1, 1.0), (2, 1.0))


def make_imu(cal=None):
    imu = ESP32IMU.__new__(ESP32IMU)
    ESP32IMU.__init__(imu, port="/dev/null", cal=cal or _Cal())
    return imu


class TestFrameFormat(unittest.TestCase):
    def test_frame_is_18_bytes(self):
        self.assertEqual(len(frame(0, 0, 0, 0, 0, 0, 0)), FRAME_LEN)

    def test_counts_scale_to_metres_per_second_squared(self):
        """1 g of counts on sensor z -> 9.81 on body z under the identity map.

        Note the SIGN: a level airframe must read -9.81 on FRD z, so an
        identity map is only correct for a sensor mounted z-DOWN. That is
        precisely what axis_map has to establish, and why it ships unverified.
        """
        imu = make_imu()
        imu._on_frame(frame(1, 0, 0, int(ACCEL_LSB_PER_G), 0, 0, 0))
        _, _, a = imu.get_state()
        np.testing.assert_allclose(a, [0, 0, G], atol=1e-3)

    def test_gyro_counts_become_radians(self):
        imu = make_imu()
        imu._on_frame(frame(1, 0, 0, 0, int(GYRO_LSB_PER_DPS * 180), 0, 0))
        _, w, _ = imu.get_state()
        self.assertAlmostEqual(w[0], math.pi, places=2)

    def test_gyro_bias_is_removed_before_scaling(self):
        class C(_Cal):
            gyro_bias = (100.0, 0.0, 0.0)
        imu = make_imu(C())
        imu._on_frame(frame(1, 0, 0, 0, 100, 0, 0))
        _, w, _ = imu.get_state()
        self.assertAlmostEqual(w[0], 0.0, places=6)

    def test_axis_map_is_applied(self):
        class C(_Cal):
            axis_map = ((1, 1.0), (0, -1.0), (2, -1.0))
        imu = make_imu(C())
        imu._on_frame(frame(1, int(ACCEL_LSB_PER_G), 0, 0, 0, 0, 0))
        _, _, a = imu.get_state()
        self.assertAlmostEqual(a[1], -G, places=3)   # sensor x -> body y, flipped

    def test_accel_scale_is_configurable(self):
        class C(_Cal):
            accel_per_g = 2077.5
        imu = make_imu(C())
        imu._on_frame(frame(1, 0, 0, 2078, 0, 0, 0))
        _, _, a = imu.get_state()
        self.assertAlmostEqual(np.linalg.norm(a), G, places=2)


class TestPerAxisAccelCalibration(unittest.TestCase):
    """This die reads 2400 counts/g on one axis and 2001 on another.

    A single scale cannot express that, and the error does not stay harmless:
    Mahony normalises the accel vector, so a common scale error cancels while
    a per-axis OFFSET tilts measured gravity and becomes standing attitude
    bias the self-level module dutifully flies into.
    """

    def test_per_axis_scale_beats_the_scalar(self):
        class C(_Cal):
            accel_per_g = 2048.0
            accel_per_g_axis = (2001.0, 2100.0, 2400.0)
        imu = make_imu(C())
        for counts, axis in ((2001, 0), (2100, 1), (2400, 2)):
            f = [0, 0, 0]
            f[axis] = counts
            imu._on_frame(frame(1, f[0], f[1], f[2], 0, 0, 0))
            _, _, a = imu.get_state()
            self.assertAlmostEqual(np.linalg.norm(a), G, places=2,
                                   msg=f"sensor axis {axis} did not read 1 g")

    def test_offset_is_removed_in_the_sensor_frame(self):
        """Offset must be subtracted BEFORE the axis map — it is a die property."""
        class C(_Cal):
            accel_offset = (0.0, 0.0, 350.0)
            accel_per_g_axis = (2048.0, 2048.0, 2048.0)
            axis_map = ((0, 1.0), (1, -1.0), (2, -1.0))
        imu = make_imu(C())
        imu._on_frame(frame(1, 2048, 0, 350, 0, 0, 0))
        _, _, a = imu.get_state()
        np.testing.assert_allclose(a, [G, 0.0, 0.0], atol=1e-3)

    def test_uncorrected_offset_is_a_real_attitude_error(self):
        """Quantifies the harm: 350 counts of z offset is ~10 deg, not noise."""
        raw = np.array([-1960.0, 0.0, 0.0])
        offset = np.array([0.0, 0.0, 350.0])
        bad = (raw + offset) / np.linalg.norm(raw + offset)
        good = raw / np.linalg.norm(raw)
        err = math.degrees(math.acos(np.clip(np.dot(bad, good), -1, 1)))
        self.assertGreater(err, 9.0)

    def test_scalar_scale_still_works_when_no_per_axis_is_given(self):
        class C(_Cal):
            accel_per_g = 2077.5
        imu = make_imu(C())
        np.testing.assert_allclose(imu.accel_scale, [2077.5] * 3)

    def test_config_dataclass_carries_the_new_fields(self):
        from companion.config import ESP32IMUCal
        c = ESP32IMUCal()
        self.assertEqual(tuple(c.accel_offset), (0.0, 0.0, 0.0))
        self.assertEqual(tuple(c.accel_per_g_axis), ())


class TestEllipsoidFit(unittest.TestCase):
    """Recover offset and scale from tumbling, with no pose ever square.

    The airframe cannot be rested on six faces, so the calibration has to work
    from arbitrary orientations. Every at-rest sample lies on a sphere of
    radius 1 g; a real part smears that into an offset ellipsoid, and its
    centre and radii ARE the calibration.
    """

    @staticmethod
    def _fit(samples):
        sys.path.insert(0, os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))
        from bringup import _fit_accel_ellipsoid
        return _fit_accel_ellipsoid(samples)

    @staticmethod
    def _tumble(offset, scale, n=400, seed=7, noise=0.0):
        rng = np.random.default_rng(seed)
        v = rng.normal(size=(n, 3))
        v /= np.linalg.norm(v, axis=1, keepdims=True)     # unit gravity, any way up
        return v * np.asarray(scale) + np.asarray(offset) + rng.normal(
            scale=noise, size=(n, 3))

    def test_recovers_a_known_offset_and_scale(self):
        off, sca = [12.0, -40.0, 350.0], [2001.0, 2100.0, 2400.0]
        got_off, got_sca, resid = self._fit(self._tumble(off, sca))
        np.testing.assert_allclose(got_off, off, atol=1.0)
        np.testing.assert_allclose(got_sca, sca, rtol=1e-3)
        self.assertLess(resid, 1e-6)

    def test_survives_realistic_sensor_noise(self):
        off, sca = [61.9, -102.0, 350.0], [2001.0, 2050.0, 2400.0]
        got_off, got_sca, _ = self._fit(self._tumble(off, sca, noise=4.0))
        np.testing.assert_allclose(got_off, off, atol=15.0)
        np.testing.assert_allclose(got_sca, sca, rtol=0.01)

    def test_output_actually_normalises_gravity(self):
        """The point of the fit: |a| must read 1 g in every orientation."""
        off, sca = [0.0, 0.0, 350.0], [2001.0, 2050.0, 2400.0]
        data = self._tumble(off, sca, n=200, seed=3)
        got_off, got_sca, _ = self._fit(data)
        mags = np.linalg.norm((data - got_off) / got_sca, axis=1)
        self.assertLess(abs(mags.max() - 1.0), 1e-3)
        self.assertLess(abs(mags.min() - 1.0), 1e-3)

    def test_coverage_of_a_motionless_board_is_near_zero(self):
        """Regression: seeded from zeros, this read 0.48 g for a still board.

        Gravity at -1960 counts made max=0, min=-1960, so the "how far did you
        turn it" number was really "how far is this from the origin" — and two
        90 s runs of an airframe nobody picked up looked half calibrated.
        """
        sys.path.insert(0, os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))
        from bringup import _turned_span
        rng = np.random.default_rng(0)
        still = np.array([-1954.0, 314.0, -268.0]) + rng.normal(scale=3.0,
                                                                size=(600, 3))
        np.testing.assert_allclose(_turned_span(still) / 2048.0, 0.0, atol=0.01)

    def test_coverage_sees_a_real_tumble(self):
        sys.path.insert(0, os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))
        from bringup import _turned_span
        tumbled = self._tumble([0.0, 0.0, 350.0], [2048.0] * 3, n=500)
        self.assertTrue(np.all(_turned_span(tumbled) / 2048.0 > 0.9))

    def test_refuses_a_one_sided_cloud(self):
        """An ellipsoid fits through any patch of its own surface.

        So a cloud confined to one octant yields a confident answer that is
        pure extrapolation off the patch. Refusing is the only safe response —
        this calibration decides which way is down.
        """
        rng = np.random.default_rng(1)
        v = np.abs(rng.normal(size=(300, 3)))       # whole cloud in one octant
        v /= np.linalg.norm(v, axis=1, keepdims=True)
        with self.assertRaises((ValueError, np.linalg.LinAlgError)) as cm:
            self._fit(v * 2048.0 + np.array([0.0, 0.0, 350.0]))
        self.assertIn("up and down", str(cm.exception))

    def test_refuses_a_single_axis_of_rotation(self):
        """Spinning about one axis only leaves that axis' offset unobserved."""
        ang = np.linspace(0, 2 * np.pi, 300)
        v = np.stack([np.cos(ang), np.sin(ang), np.zeros_like(ang)], axis=1)
        with self.assertRaises((ValueError, np.linalg.LinAlgError)):
            self._fit(v * 2048.0 + np.array([0.0, 0.0, 350.0]))


class TestAxisMapAgainstFC(unittest.TestCase):
    """The FC is a second sensor on the same airframe — an independent witness.

    Its estimate comes from a different die calibrated by different software,
    so it cannot agree with a wrong ESP32 axis map by construction. That is
    what makes it worth an MSP transaction, and what the level/nose-up/
    roll-right procedure could never provide: that one only ever checks the
    ESP32 against the operator's idea of "nose up".
    """

    @staticmethod
    def _tools():
        sys.path.insert(0, os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))
        import bringup
        return bringup

    def test_level_gravity_points_down_in_body(self):
        b = self._tools()
        np.testing.assert_allclose(
            b._gravity_in_body_from_fc({"roll": 0.0, "pitch": 0.0}),
            [0, 0, 1], atol=1e-9)

    def test_nose_up_moves_gravity_to_minus_x(self):
        b = self._tools()
        v = b._gravity_in_body_from_fc({"roll": 0.0, "pitch": 30.0})
        self.assertAlmostEqual(v[0], -0.5, places=6)
        self.assertAlmostEqual(v[2], math.cos(math.radians(30)), places=6)

    def test_right_roll_moves_gravity_to_plus_y(self):
        b = self._tools()
        v = b._gravity_in_body_from_fc({"roll": 30.0, "pitch": 0.0})
        self.assertAlmostEqual(v[1], 0.5, places=6)

    def test_gravity_direction_is_always_unit_length(self):
        b = self._tools()
        for r in (-80, -35, 0, 12, 70):
            for p in (-70, -20, 0, 45, 85):
                v = b._gravity_in_body_from_fc({"roll": float(r), "pitch": float(p)})
                self.assertAlmostEqual(np.linalg.norm(v), 1.0, places=9)

    def test_kabsch_recovers_a_known_mounting(self):
        """Full round trip: bolt the board at a known 90 deg rotation, find it."""
        b = self._tools()
        truth = np.array([[0.0, 1.0, 0.0],       # sensor y -> body x
                          [-1.0, 0.0, 0.0],      # sensor x -> body y, flipped
                          [0.0, 0.0, 1.0]])
        rng = np.random.default_rng(11)
        S = rng.normal(size=(300, 3))
        S /= np.linalg.norm(S, axis=1, keepdims=True)
        B = (truth @ S.T).T
        U, _, Vt = np.linalg.svd(S.T @ B)
        d = np.sign(np.linalg.det(Vt.T @ U.T))
        M = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
        np.testing.assert_allclose(M, truth, atol=1e-9)
        rows, quality = b._snap_to_signed_permutation(M)
        self.assertEqual(rows, [(1, 1.0), (0, -1.0), (2, 1.0)])
        self.assertGreater(quality, 0.99)

    def test_snap_never_assigns_one_sensor_axis_twice(self):
        """A duplicated source axis is a silently unusable map — reject by design."""
        b = self._tools()
        rng = np.random.default_rng(5)
        for _ in range(50):
            rows, _ = b._snap_to_signed_permutation(rng.normal(size=(3, 3)))
            self.assertEqual(len({r[0] for r in rows}), 3)

    def test_snap_confidence_falls_for_a_skew_mounting(self):
        """45 deg between two axes must NOT be reported as a clean map."""
        b = self._tools()
        c = math.cos(math.radians(45))
        skew = np.array([[c, c, 0.0], [-c, c, 0.0], [0.0, 0.0, 1.0]])
        _, quality = b._snap_to_signed_permutation(skew)
        self.assertLess(quality, 0.80)


class TestSequenceAndIntegrity(unittest.TestCase):
    def test_dropped_samples_are_counted(self):
        imu = make_imu()
        imu._on_frame(frame(1, 0, 0, 0, 0, 0, 0))
        imu._on_frame(frame(5, 0, 0, 0, 0, 0, 0))     # 2,3,4 lost
        self.assertEqual(imu.drops, 3)

    def test_sequence_wraps_cleanly(self):
        imu = make_imu()
        imu._on_frame(frame(255, 0, 0, 0, 0, 0, 0))
        imu._on_frame(frame(0, 0, 0, 0, 0, 0, 0))
        self.assertEqual(imu.drops, 0)

    def test_checksum_matches_the_firmware(self):
        """XOR over seq+payload — must agree with esp32/main.py byte for byte."""
        f = frame(7, 1, 2, 3, 4, 5, 6)
        x = 0
        for b in f[2:17]:
            x ^= b
        self.assertEqual(x, f[17])

    def test_corrupt_frame_has_a_different_checksum(self):
        self.assertNotEqual(frame(1, 1, 2, 3, 4, 5, 6)[17],
                            frame(1, 1, 2, 3, 4, 5, 6, corrupt=True)[17])


class TestBridgeRecovery(unittest.TestCase):
    """A wedged bridge is silent, not slow — and it stays wedged until reset.

    MEASURED 2026-08-04: after a companion reboot the ESP32 sat enumerated and
    emitted nothing for 19 minutes. Opening and reading the port did not help;
    only a chip reset did. The reader has to do that itself, because the
    symptom otherwise is a control loop running happily on a frozen attitude.
    """

    class FakeSerial:
        def __init__(self, feed=b""):
            self.feed = feed
            self.closed = False

        @property
        def in_waiting(self):
            return len(self.feed)

        def read(self, n):
            out, self.feed = self.feed[:n], self.feed[n:]
            return out

        def close(self):
            self.closed = True

    def _imu(self, **kw):
        imu = make_imu()
        imu.recover_after_s = kw.get("recover_after_s", 0.05)
        imu.port = "/dev/null"
        return imu

    def _patched(self, imu, calls, reset=None):
        """Swap out the two things that touch real hardware."""
        import companion.imu_esp32 as m

        def fake_open(self):
            calls.append("open")
            self._ser = TestBridgeRecovery.FakeSerial()

        def fake_reset(ser, **k):
            calls.append("reset")
            if reset:
                reset()

        saved = (m.pulse_reset, m.ESP32IMU._open)
        m.pulse_reset, m.ESP32IMU._open = fake_reset, fake_open
        return saved

    @staticmethod
    def _restore(saved):
        import companion.imu_esp32 as m
        m.pulse_reset, m.ESP32IMU._open = saved

    def test_port_is_opened_before_the_chip_is_reset(self):
        """Order IS the fix, not an accident.

        cdc_acm only drains the USB endpoint while the tty is open, so a
        bridge that boots before anything opens the port streams into a closed
        door and wedges. Resetting through a handle we already hold makes that
        race impossible; reset-then-open would recreate it exactly.
        """
        imu = self._imu()
        calls = []
        saved = self._patched(imu, calls)
        try:
            imu._ser = self.FakeSerial()
            imu._recover()
        finally:
            self._restore(saved)
        self.assertEqual(calls, ["open", "reset"])
        self.assertEqual(imu.recoveries, 1)

    def test_recovery_does_not_charge_the_restart_to_drops(self):
        """The bridge restarts seq at 0; that is not 68 lost samples."""
        imu = self._imu()
        imu._on_frame(frame(200, 0, 0, 0, 0, 0, 0))
        saved = self._patched(imu, [])
        try:
            imu._recover()
        finally:
            self._restore(saved)
        self.assertIsNone(imu._last_seq)
        imu._on_frame(frame(0, 0, 0, 0, 0, 0, 0))
        self.assertEqual(imu.drops, 0)

    def test_a_failed_reset_does_not_raise_or_spin(self):
        imu = self._imu()
        saved = self._patched(
            imu, [], reset=lambda: (_ for _ in ()).throw(OSError("gone")))
        try:
            t0 = time.monotonic()
            imu._recover()                      # must swallow it
            self.assertGreater(time.monotonic() - t0, 0.5)   # backed off
        finally:
            self._restore(saved)
        self.assertEqual(imu.recoveries, 1)

    def test_a_booting_bridge_is_not_mistaken_for_a_wedged_one(self):
        """After a reset the bridge is legitimately silent for ~STARTUP_DELAY_S.

        Without a grace window the reader would call that a wedge, reset it
        again, and loop forever — the recovery would become the outage.
        """
        from companion.imu_esp32 import BOOT_GRACE_S
        imu = self._imu()
        calls = []
        saved = self._patched(imu, calls)
        try:
            imu._recover()
        finally:
            self._restore(saved)
        self.assertGreater(imu._grace_until - time.monotonic(), BOOT_GRACE_S - 1)
        imu._ser = self.FakeSerial()
        imu._last_rx = 0.0                      # ancient: looks silent
        imu._stop.set()
        imu._loop()                             # one pass
        self.assertEqual(imu.recoveries, 1)     # still 1 — no second reset

    def test_recovery_can_be_disabled(self):
        imu = make_imu()
        imu.recover_after_s = 0
        imu._ser = self.FakeSerial()
        imu._last_rx = 0.0                      # ancient
        imu._stop.set()                         # one pass only
        imu._loop()
        self.assertEqual(imu.recoveries, 0)

    def test_stats_expose_the_recovery_count(self):
        imu = make_imu()
        self.assertIn("recoveries", imu.stats())


class TestMahony(unittest.TestCase):
    """The filter must converge to the attitude implied by measured gravity."""

    def converge(self, accel, n=4000, kp=2.0, ki=0.05):
        f = Mahony(kp, ki)
        for _ in range(n):
            f.update(np.zeros(3), np.asarray(accel, float), 1.0 / 200.0)
        return f.q

    def tilt_error_deg(self, q, accel):
        a = np.asarray(accel, float)
        pred = -q_to_R(q)[2, :]
        meas = a / np.linalg.norm(a)
        return math.degrees(math.acos(np.clip(np.dot(pred, meas), -1, 1)))

    def test_level_converges_to_level(self):
        q = self.converge([0, 0, -G])
        roll, pitch, _ = q_to_euler(q)
        self.assertAlmostEqual(roll, 0.0, places=1)
        self.assertAlmostEqual(pitch, 0.0, places=1)

    def test_converges_for_arbitrary_tilts(self):
        for accel in ([0, -G * 0.5, -G * 0.866],
                      [G * 0.5, 0, -G * 0.866],
                      [-2.0, 1.0, -9.4]):
            q = self.converge(accel)
            self.assertLess(self.tilt_error_deg(q, accel), 0.5,
                            f"did not converge for accel={accel}")

    def test_nose_up_gives_positive_pitch(self):
        """Nose up 30 deg: specific force gains +x in body FRD."""
        q = self.converge([G * 0.5, 0, -G * 0.866])
        _, pitch, _ = q_to_euler(q)
        self.assertGreater(pitch, 25.0)
        self.assertLess(pitch, 35.0)

    def test_right_roll_gives_positive_roll(self):
        """Right side down: specific force gains -y in body FRD."""
        q = self.converge([0, -G * 0.5, -G * 0.866])
        roll, _, _ = q_to_euler(q)
        self.assertGreater(roll, 25.0)
        self.assertLess(roll, 35.0)

    def test_gyro_only_integrates(self):
        f = Mahony(kp=0.0, ki=0.0)
        for _ in range(200):
            f.update([0.0, 0.0, math.radians(90)], [0.0, 0.0, 0.0], 1.0 / 200.0)
        self.assertAlmostEqual(q_to_euler(f.q)[2], 90.0, places=0)

    def test_quaternion_stays_normalised(self):
        f = Mahony()
        for i in range(500):
            f.update([0.1, -0.2, 0.3], [0.5, -1.0, -9.6], 1.0 / 200.0)
        self.assertAlmostEqual(np.linalg.norm(f.q), 1.0, places=9)


class TestConfigWiring(unittest.TestCase):
    def test_repo_config_has_measured_calibration(self):
        from companion.config import load
        repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        cfg = load(os.path.join(repo, "config", "vehicle.yaml"))
        self.assertTrue(cfg.imu32.enabled)
        self.assertNotEqual(tuple(cfg.imu32.gyro_bias), (0.0, 0.0, 0.0),
                            "gyro bias must be measured — it is ~9 dps on this part")
        self.assertGreater(cfg.imu32.accel_per_g, 1900)
        self.assertLess(cfg.imu32.accel_per_g, 2200)

    def test_unverified_axis_map_is_reported(self):
        from companion.config import load
        repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        cfg = load(os.path.join(repo, "config", "vehicle.yaml"))
        if not cfg.imu32.verified:
            self.assertTrue(any("esp32" in p for p in cfg.unverified()))

    def test_factory_knows_esp32(self):
        from companion.imu_driver import make_imu
        with self.assertRaises(ValueError):
            make_imu("nonsense")


if __name__ == "__main__":
    unittest.main()
