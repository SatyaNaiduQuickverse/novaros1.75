"""Connector-side boundary handler. No hardware, no real subprocess.

The full spawn/isolation path is exercised separately against real hardware;
these pin the logic that decides whether to launch, and every path that must
end in an abort.
"""

import hashlib
import os
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from companion.config import GuidanceConfig  # noqa: E402
from companion.guidance_proxy import (  # noqa: E402
    ABORT_HOST_DIED, ABORT_HOST_PROTOCOL, ABORT_HOST_TIMEOUT,
    GuidanceProxy, so_digest, reachable_serial_devices,
)
from tests.test_fc_link import make_link  # noqa: E402


class FakePipe:
    """Stands in for the multiprocessing Connection."""

    def __init__(self, replies=None, poll_ok=True):
        self.sent = []
        self.replies = list(replies or [])
        self.poll_ok = poll_ok
        self.closed = False
        self.eof = False

    def send(self, msg):
        if self.closed:
            raise BrokenPipeError("pipe closed")
        self.sent.append(msg)

    def poll(self, timeout=None):
        return self.poll_ok and bool(self.replies)

    def recv(self):
        if self.eof:
            raise EOFError
        return self.replies.pop(0)

    def close(self):
        self.closed = True


class FakeProc:
    def __init__(self, alive=True):
        self._alive = alive

    def is_alive(self):
        return self._alive

    def terminate(self):
        self._alive = False

    def join(self, t=None):
        pass

    def kill(self):
        self._alive = False


class _IMU:
    def get_state(self):
        import numpy as np
        return (np.array([1.0, 0, 0, 0]), np.zeros(3),
                np.array([0.0, 0.0, -9.80665]))


class _Vision:
    def bearing(self, t):
        return None


def make_proxy(replies=None, poll_ok=True, alive=True, **cfgkw):
    fc = make_link()
    cfg = GuidanceConfig(enabled=True, cert_sha256="a" * 64, **cfgkw)
    p = GuidanceProxy(fc, _IMU(), _Vision(), cfg=cfg)
    p._pipe = FakePipe(replies, poll_ok)
    p._proc = FakeProc(alive)
    return fc, p


class TestDigest(unittest.TestCase):
    def test_single_file_matches_sha256sum(self):
        """Must equal what the build prints, or pinning is unusable."""
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "libguidance.so")
            blob = os.urandom(2048)
            with open(path, "wb") as f:
                f.write(blob)
            digest, files = so_digest(d)
            self.assertEqual(digest, hashlib.sha256(blob).hexdigest())
            self.assertEqual(files, [path])
            out = subprocess.run(["sha256sum", path], capture_output=True,
                                 text=True).stdout.split()[0]
            self.assertEqual(digest, out)

    def test_missing_binary_raises(self):
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(FileNotFoundError):
                so_digest(d)


class TestLaunchRefusals(unittest.TestCase):
    def test_refuses_unpinned(self):
        fc, p = make_proxy()
        p.cfg.cert_sha256 = ""
        with self.assertRaises(ValueError):
            p.start()

    def test_refuses_hash_mismatch(self):
        with tempfile.TemporaryDirectory() as d:
            with open(os.path.join(d, "g.so"), "wb") as f:
                f.write(b"not the certified build")
            fc, p = make_proxy(so_dir=d)
            with self.assertRaises(PermissionError) as cm:
                p.start()
            self.assertIn("hash mismatch", str(cm.exception))


class TestTickPaths(unittest.TestCase):
    def test_clamps_on_the_boundary(self):
        """Desired sticks arrive unclamped; the envelope is applied here."""
        fc, p = make_proxy(replies=[("cmd", (2400, 300, -800, 2000))])
        p.step(0.0)
        roll, pitch, thr, yaw = fc.msp.rc_frames[-1]   # wire order is AETR
        self.assertEqual(thr, fc.limits.thr_cap)
        self.assertEqual(roll, 1500 + fc.limits.max_deflect)
        self.assertEqual(pitch, 1500 - fc.limits.max_deflect)
        self.assertEqual(yaw, 1500 - fc.limits.max_deflect)

    def test_timeout_aborts(self):
        fc, p = make_proxy(replies=[("cmd", (1500, 1500, 1500, 1000))],
                           poll_ok=False)
        p.step(0.0)
        self.assertIn(ABORT_HOST_TIMEOUT, fc.abort_reason)

    def test_dead_host_aborts(self):
        fc, p = make_proxy(alive=False)
        p.step(0.0)
        self.assertIn(ABORT_HOST_DIED, fc.abort_reason)

    def test_eof_aborts(self):
        fc, p = make_proxy(replies=[("cmd", (1500,) * 4)])
        p._pipe.eof = True
        p.step(0.0)
        self.assertIn(ABORT_HOST_DIED, fc.abort_reason)

    def test_malformed_reply_aborts(self):
        fc, p = make_proxy(replies=["not-a-tuple"])
        p.step(0.0)
        self.assertIn(ABORT_HOST_PROTOCOL, fc.abort_reason)

    def test_wrong_tag_aborts(self):
        fc, p = make_proxy(replies=[("nope", (1500,) * 4)])
        p.step(0.0)
        self.assertIn(ABORT_HOST_PROTOCOL, fc.abort_reason)

    def test_no_frames_sent_after_abort(self):
        fc, p = make_proxy(alive=False)
        p.step(0.0)
        before = len(fc.msp.rc_frames)
        p.step(0.02)
        p.step(0.04)
        self.assertEqual(len(fc.msp.rc_frames), before)

    def test_bearing_none_is_forwarded_as_none(self):
        fc, p = make_proxy(replies=[("cmd", (1500,) * 4)])
        p.step(0.0)
        tag, t, q, w, accel, bearing = p._pipe.sent[-1]
        self.assertEqual(tag, "tick")
        self.assertIsNone(bearing)
        self.assertIsInstance(q, list)   # plain lists, no numpy across the wire


class TestEngage(unittest.TestCase):
    def test_engage_commands_idle_first(self):
        """Nothing may be inherited from an earlier run — least of all throttle."""
        fc, p = make_proxy()
        fc.set_stick(roll=1580, throttle=1090)
        p.engage()
        roll, pitch, thr, yaw = fc.msp.rc_frames[-1]
        self.assertEqual(thr, 1000)
        self.assertEqual(roll, 1500)

    def test_engage_sends_v0_p0(self):
        fc, p = make_proxy()
        p.engage()
        self.assertEqual(p._pipe.sent[-1], ("engage", [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]))


class TestIsolationProbe(unittest.TestCase):
    def test_this_process_reports_its_own_serial_fds(self):
        """The probe the spawned child uses to prove it cannot reach the FC."""
        self.assertIsInstance(reachable_serial_devices(), list)

    def test_probe_sees_an_open_serial_device(self):
        import glob
        ports = glob.glob("/dev/serial/by-id/*")
        if not ports:
            self.skipTest("no serial device present")
        import serial
        try:
            s = serial.Serial(ports[0], 115200, timeout=0.01)
        except Exception:
            self.skipTest("serial device busy")
        try:
            self.assertTrue(any("tty" in d for d in reachable_serial_devices()),
                            "probe failed to notice an open serial port")
        finally:
            s.close()
        self.assertFalse(reachable_serial_devices())


if __name__ == "__main__":
    unittest.main()
