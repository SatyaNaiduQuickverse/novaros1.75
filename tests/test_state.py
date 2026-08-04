"""Attitude/IMU maths, IMU decoding, the vision adapter, and the config loader."""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from companion.config import CameraCal, IMUCal, load  # noqa: E402
from companion.fc_link import decode_raw_imu  # noqa: E402
from companion.math_utils import (  # noqa: E402
    euler_to_q, q_to_R, q_to_euler, q_mult, q_conj, q_normalize,
)
from companion.imu_driver import FakeIMU  # noqa: E402
from companion.vision_adapter import VisionAdapter  # noqa: E402

G = 9.80665


class TestQuaternions(unittest.TestCase):
    def test_R_is_orthonormal(self):
        for angles in [(0, 0, 0), (10, -20, 30), (89, 5, 359), (-45, 45, 180)]:
            R = q_to_R(euler_to_q(*angles))
            np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-9)
            self.assertAlmostEqual(np.linalg.det(R), 1.0, places=9)

    def test_euler_round_trip(self):
        for angles in [(0, 0, 0), (10, -20, 30), (-45, 30, 120)]:
            back = q_to_euler(euler_to_q(*angles))
            for a, b in zip(angles, back):
                self.assertAlmostEqual(a, b, places=6)

    def test_yaw_rotates_north_to_east(self):
        """Yaw +90 deg must send body-forward to world east (NED)."""
        v = q_to_R(euler_to_q(0, 0, 90)) @ np.array([1.0, 0, 0])
        np.testing.assert_allclose(v, [0, 1, 0], atol=1e-9)

    def test_pitch_up_lifts_the_nose(self):
        """Pitch +30 deg: body-forward gains a negative (upward) world z."""
        v = q_to_R(euler_to_q(0, 30, 0)) @ np.array([1.0, 0, 0])
        self.assertLess(v[2], 0)

    def test_conjugate_inverts(self):
        q = euler_to_q(15, -25, 200)
        np.testing.assert_allclose(q_mult(q, q_conj(q)), [1, 0, 0, 0], atol=1e-9)

    def test_normalize(self):
        self.assertAlmostEqual(np.linalg.norm(q_normalize([2.0, 0, 0, 0])), 1.0)


class TestIMUDecode(unittest.TestCase):
    def setUp(self):
        self.cal = IMUCal()  # default board_to_frd = (x, -y, -z)

    def test_level_reads_minus_one_g_on_z(self):
        """Level airframe: specific force points up, so FRD z is -9.81."""
        raw = [0, 0, int(self.cal.acc_per_g), 0, 0, 0]   # +1 g on board z (up)
        acc, _ = decode_raw_imu(raw, self.cal)
        np.testing.assert_allclose(acc, [0, 0, -G], atol=1e-6)

    def test_axis_map_is_applied(self):
        raw = [int(self.cal.acc_per_g), int(self.cal.acc_per_g), 0, 0, 0, 0]
        acc, _ = decode_raw_imu(raw, self.cal)
        self.assertAlmostEqual(acc[0], G, places=5)     # x passes through
        self.assertAlmostEqual(acc[1], -G, places=5)    # y is flipped

    def test_gyro_dps_becomes_radians(self):
        _, gyro = decode_raw_imu([0, 0, 0, 180, 0, 0], self.cal)
        self.assertAlmostEqual(gyro[0], np.pi, places=5)

    def test_legacy_lsb_scaling(self):
        cal = IMUCal(gyro_units="lsb2000")
        _, gyro = decode_raw_imu([0, 0, 0, 164, 0, 0], cal)
        self.assertAlmostEqual(gyro[0], np.deg2rad(10.0), places=5)

    def test_acc_scale_is_configurable(self):
        cal = IMUCal(acc_per_g=256.0)
        acc, _ = decode_raw_imu([0, 0, 256, 0, 0, 0], cal)
        self.assertAlmostEqual(acc[2], -G, places=5)


class TestVisionAdapter(unittest.TestCase):
    def setUp(self):
        self.cal = CameraCal()
        self.level = lambda: euler_to_q(0, 0, 0)

    def test_no_source_means_no_measurement(self):
        va = VisionAdapter(self.cal, get_attitude_fn=self.level)
        self.assertIsNone(va.bearing(0.0))

    def test_centre_pixel_is_the_boresight(self):
        box = ([self.cal.cx - 20, self.cal.cy - 20, 40, 40], 0.9)
        va = VisionAdapter(self.cal, get_bbox_fn=lambda: box,
                           get_attitude_fn=self.level)
        np.testing.assert_allclose(va.bearing(0.0).u_world, [1, 0, 0], atol=1e-9)

    def test_object_right_of_centre_bears_right(self):
        box = ([self.cal.cx + 200, self.cal.cy - 20, 40, 40], 0.9)
        va = VisionAdapter(self.cal, get_bbox_fn=lambda: box,
                           get_attitude_fn=self.level)
        u = va.bearing(0.0).u_world
        self.assertGreater(u[1], 0, "object right of centre must bear to body +y")

    def test_object_above_centre_bears_up(self):
        box = ([self.cal.cx - 20, self.cal.cy - 200, 40, 40], 0.9)
        va = VisionAdapter(self.cal, get_bbox_fn=lambda: box,
                           get_attitude_fn=self.level)
        u = va.bearing(0.0).u_world
        self.assertLess(u[2], 0, "object above centre must bear upward (NED -z)")

    def test_attitude_rotates_the_bearing(self):
        box = ([self.cal.cx - 20, self.cal.cy - 20, 40, 40], 0.9)
        va = VisionAdapter(self.cal, get_bbox_fn=lambda: box,
                           get_attitude_fn=lambda: euler_to_q(0, 0, 90))
        np.testing.assert_allclose(va.bearing(0.0).u_world, [0, 1, 0], atol=1e-9)

    def test_bearing_is_a_unit_vector(self):
        box = ([10.0, 10.0, 40, 40], 0.5)
        va = VisionAdapter(self.cal, get_bbox_fn=lambda: box,
                           get_attitude_fn=self.level)
        self.assertAlmostEqual(np.linalg.norm(va.bearing(0.0).u_world), 1.0, places=6)

    def test_range_hint_suppressed_for_tiny_boxes(self):
        small = ([600.0, 300.0, 10, 10], 0.9)
        big = ([600.0, 300.0, 100, 100], 0.9)
        va = VisionAdapter(self.cal, get_bbox_fn=lambda: small,
                           get_attitude_fn=self.level)
        self.assertIsNone(va.bearing(0.0).range_m)
        va.get_bbox = lambda: big
        self.assertIsNotNone(va.bearing(0.0).range_m)

    def test_range_shrinks_as_the_box_grows(self):
        va = VisionAdapter(self.cal, get_bbox_fn=lambda: ([600.0, 300.0, 50, 50], 0.9),
                           get_attitude_fn=self.level)
        near = va.bearing(0.0).range_m
        va.get_bbox = lambda: ([600.0, 300.0, 200, 200], 0.9)
        self.assertLess(va.bearing(0.0).range_m, near)

    def test_fake_mode_needs_no_tracker(self):
        va = VisionAdapter(self.cal, get_attitude_fn=self.level, fake=True)
        self.assertIsNotNone(va.bearing(0.0))


class TestFakeIMU(unittest.TestCase):
    def test_contract(self):
        q, w, acc = FakeIMU().start().get_state()
        self.assertEqual(q.shape, (4,))
        self.assertEqual(w.shape, (3,))
        np.testing.assert_allclose(acc, [0, 0, -G], atol=1e-6)


class TestConfig(unittest.TestCase):
    def test_defaults_load_without_a_file(self):
        cfg = load("/nonexistent/vehicle.yaml")
        self.assertEqual(cfg.limits.thr_cap, 1100)
        self.assertEqual(cfg.channels.arm_index, 8)

    def test_repo_config_parses(self):
        repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        cfg = load(os.path.join(repo, "config", "vehicle.yaml"))
        # Either "auto" or a full /dev/serial/by-id/ path — never a bare
        # ttyACM number, which an ESP32 on this Pi can steal.
        self.assertTrue(cfg.fc.port == "auto"
                        or cfg.fc.port.startswith("/dev/serial/by-id/"),
                        f"unsafe port setting: {cfg.fc.port!r}")
        self.assertEqual(cfg.limits.profile, "bench")
        self.assertEqual(len(cfg.imu.board_to_frd), 3)

    def test_unverified_calibrations_are_reported(self):
        self.assertEqual(len(load("/nonexistent.yaml").unverified()), 2)


if __name__ == "__main__":
    unittest.main()
