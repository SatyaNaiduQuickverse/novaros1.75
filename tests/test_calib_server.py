"""The calibration UI's config writer.

This is the only code in the repo that edits ``config/vehicle.yaml``, so it
gets tested harder than its size suggests. That file is the audit trail — every
value carries the measurement and the date behind it — and a writer that
quietly drops the commentary, or corrupts a neighbouring block, would cost more
than the calibration is worth.
"""

import os
import shutil
import sys
import tempfile
import unittest

import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.calib_server import apply_to_yaml  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VEHICLE = os.path.join(REPO, "config", "vehicle.yaml")


class TestApplyToYaml(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.path = os.path.join(self.dir, "vehicle.yaml")
        shutil.copy2(VEHICLE, self.path)
        with open(self.path) as f:
            self.before = f.read()

    def tearDown(self):
        shutil.rmtree(self.dir, ignore_errors=True)

    def load(self):
        with open(self.path) as f:
            return yaml.safe_load(f)

    def test_scalar_and_list_values_land(self):
        apply_to_yaml(self.path, {"accel_offset": [12.0, -40.0, 350.0],
                                  "accel_per_g": 2167.0})
        d = self.load()["imu32"]
        self.assertEqual(d["accel_offset"], [12.0, -40.0, 350.0])
        self.assertEqual(d["accel_per_g"], 2167.0)

    def test_replacing_a_block_sequence_leaves_no_orphans(self):
        """axis_map is written as '- [0, +1.0]' over three lines.

        Replacing only the 'axis_map:' line leaves those items orphaned and the
        file stops parsing entirely. Caught for real during development.
        """
        apply_to_yaml(self.path, {"axis_map": [[1, 1.0], [0, -1.0], [2, 1.0]]})
        d = self.load()["imu32"]                    # would raise if orphaned
        self.assertEqual(d["axis_map"], [[1, 1.0], [0, -1.0], [2, 1.0]])
        with open(self.path) as f:
            body = f.read()
        self.assertNotIn("- [0, +1.0]\n    - [1, +1.0]", body)

    def test_only_the_imu32_block_changes(self):
        before = self.load()
        apply_to_yaml(self.path, {"accel_per_g": 1234.0})
        after = self.load()
        self.assertNotEqual(after["imu32"], before["imu32"])
        for block in ("fc", "channels", "imu", "camera", "module"):
            self.assertEqual(after[block], before[block],
                             f"{block} must not be touched")

    def test_comments_survive(self):
        """A yaml round-trip would delete every one of them."""
        apply_to_yaml(self.path, {"accel_per_g": 1234.0, "verified": "true"})
        with open(self.path) as f:
            after = f.read()
        self.assertGreaterEqual(after.count("#"), self.before.count("#"))
        self.assertIn("MEASURED", after)

    def test_untouched_keys_keep_their_values(self):
        before = self.load()["imu32"]
        apply_to_yaml(self.path, {"accel_per_g": 1234.0})
        after = self.load()["imu32"]
        for key in ("gyro_bias", "kp", "ki", "enabled", "baud"):
            self.assertEqual(after[key], before[key])

    def test_missing_keys_are_inserted_not_dropped(self):
        apply_to_yaml(self.path, {"accel_per_g_axis": [2001.0, 2100.0, 2400.0]})
        self.assertEqual(self.load()["imu32"]["accel_per_g_axis"],
                         [2001.0, 2100.0, 2400.0])

    def test_a_backup_is_kept(self):
        backup = apply_to_yaml(self.path, {"accel_per_g": 1.0})
        self.assertTrue(os.path.exists(backup))
        with open(backup) as f:
            self.assertEqual(f.read(), self.before)

    def test_result_still_loads_through_the_real_loader(self):
        apply_to_yaml(self.path, {"accel_offset": [1.0, 2.0, 3.0],
                                  "accel_per_g_axis": [2001.0, 2100.0, 2400.0]})
        apply_to_yaml(self.path, {"axis_map": [[2, -1.0], [0, 1.0], [1, -1.0]],
                                  "verified": "true"})
        from companion.config import load
        cfg = load(self.path)
        self.assertEqual(cfg.imu32.axis_map, ((2, -1.0), (0, 1.0), (1, -1.0)))
        self.assertEqual(cfg.imu32.accel_per_g_axis, (2001.0, 2100.0, 2400.0))
        self.assertTrue(cfg.imu32.verified)
        self.assertEqual(cfg.imu32.accel_offset, (1.0, 2.0, 3.0))

    def test_repeated_application_is_stable(self):
        for _ in range(3):
            apply_to_yaml(self.path, {"accel_per_g": 2000.0,
                                      "axis_map": [[0, 1.0], [1, 1.0], [2, 1.0]]})
        d = self.load()["imu32"]
        self.assertEqual(d["accel_per_g"], 2000.0)
        self.assertEqual(d["axis_map"], [[0, 1.0], [1, 1.0], [2, 1.0]])

    def test_refuses_a_file_with_no_imu32_block(self):
        path = os.path.join(self.dir, "bare.yaml")
        with open(path, "w") as f:
            f.write("fc:\n  baud: 115200\n")
        with self.assertRaises(RuntimeError):
            apply_to_yaml(path, {"accel_per_g": 1.0})


if __name__ == "__main__":
    unittest.main()
