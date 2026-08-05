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
        # imu, camera, and unit identity — a default config is bound to no
        # hardware, so it cannot know whose calibration it is carrying.
        pending = load("/nonexistent.yaml").unverified()
        self.assertEqual(len(pending), 3)
        self.assertTrue(any("unit identity" in p for p in pending))


if __name__ == "__main__":
    unittest.main()


class TestBenchRecorderKwargs(unittest.TestCase):
    """`event(name, **kw)` collides with a kw called `name`.

    The motors harness called `rec.event("move", name=name)`, which is a plain
    TypeError on the FIRST move of the sequence. It shipped because the ACRO
    gate failed before reaching the loop every previous run, so everything past
    that gate had never executed once — on a test that arms the aircraft.
    A gate that always fails hides the code behind it.
    """

    def _rec(self):
        import tempfile
        sys.path.insert(0, os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))
        from bringup import BenchRecorder
        return BenchRecorder("unittest", fc=None, run_dir=tempfile.mkdtemp())

    def test_event_accepts_the_kwargs_the_motor_sequence_uses(self):
        rec = self._rec()
        try:
            rec.event("move", move="ROLL RIGHT")
            rec.event("move_end", move="ROLL RIGHT", motors=[1040] * 4,
                      rc=[1600, 1500, 1500, 1060])
            rec.event("armed", wait_s=1.2, motors=[1043] * 4)
            rec.event("stream_start", limits="bench")
        finally:
            rec.close()

    def test_a_kwarg_called_name_still_collides(self):
        """Pin the trap itself, so nobody reintroduces it elsewhere."""
        rec = self._rec()
        try:
            with self.assertRaises(TypeError):
                rec.event("move", name="ROLL RIGHT")
        finally:
            rec.close()


class TestUnitIdentity(unittest.TestCase):
    """A config must know which airframe it was measured on.

    Every calibration is per-unit: gyro bias and accel offsets belong to a
    specific die, the mount matrix to a specific mounting. Deploy one unit's
    config to another and it inherits someone else's idea of which way is down
    — and nothing downstream can tell, because the numbers are all plausible,
    the tests pass, and preflight is green. Binding turns that silent failure
    into a refusal at connect, which is the only place it can still be caught.
    """

    def test_an_unbound_config_says_so(self):
        pending = load("/nonexistent.yaml").unverified()
        self.assertTrue(any("unit identity" in p for p in pending))

    def test_the_repo_config_is_bound(self):
        repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        cfg = load(os.path.join(repo, "config", "vehicle.yaml"))
        self.assertTrue(cfg.unit.id, "commissioned units must be bound")
        self.assertRegex(cfg.unit.fc_mcu_id, r"^[0-9a-f]{24}$")
        self.assertRegex(cfg.unit.esp32_mac,
                         r"^[0-9A-F]{2}(:[0-9A-F]{2}){5}$")
        self.assertNotIn("unit identity",
                         " ".join(cfg.unit_pending() if hasattr(cfg, "unit_pending")
                                  else cfg.unverified()))

    def test_a_wrong_fc_is_refused_not_warned(self):
        from companion.fc_link import FCLink
        from companion.msp import MSPError
        cfg = load()
        cfg.unit.fc_mcu_id = "ffffffffffffffffffffffff"
        cfg.unit.enforce = True
        fc = FCLink.__new__(FCLink)
        FCLink.__init__(fc, cfg)
        fc.msp = type("M", (), {"uid": staticmethod(lambda: "0022004c3235511137383433")})()
        with self.assertRaises(MSPError) as cm:
            fc._check_unit_identity()
        self.assertIn("WRONG AIRFRAME", str(cm.exception))

    def test_enforce_false_downgrades_to_a_loud_log(self):
        """An escape hatch, but never the default — the default must refuse."""
        from companion.fc_link import FCLink
        cfg = load()
        cfg.unit.fc_mcu_id = "ffffffffffffffffffffffff"
        cfg.unit.enforce = False
        fc = FCLink.__new__(FCLink)
        FCLink.__init__(fc, cfg)
        fc.msp = type("M", (), {"uid": staticmethod(lambda: "deadbeef")})()
        fc._check_unit_identity()          # must not raise
        from companion.config import UnitConfig
        self.assertTrue(UnitConfig().enforce, "enforce must default ON")

    def test_esp32_mac_comes_out_of_the_by_id_path(self):
        from companion.imu_esp32 import ESP32IMU
        path = ("/dev/serial/by-id/"
                "usb-Espressif_USB_JTAG_serial_debug_unit_8C:FD:49:11:35:50-if00")
        self.assertEqual(ESP32IMU.mac_from_port(path), "8C:FD:49:11:35:50")
        self.assertEqual(ESP32IMU.mac_from_port("/dev/ttyACM0"), "")
