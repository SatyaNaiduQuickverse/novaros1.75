"""MSP framing against a fake serial port. No hardware needed."""

import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from companion import msp as msp_mod  # noqa: E402
from companion.msp import MSP, MSPError, MSPTimeout  # noqa: E402


def frame(cmd, payload=b"", direction=b">", corrupt_crc=False):
    size = len(payload)
    crc = size ^ cmd
    for b in payload:
        crc ^= b
    if corrupt_crc:
        crc ^= 0xFF
    return b"$M" + direction + bytes([size, cmd]) + payload + bytes([crc])


def jumbo(cmd, payload):
    size_bytes = struct.pack("<H", len(payload))
    crc = 0xFF ^ cmd
    for b in size_bytes + payload:
        crc ^= b
    return b"$M>" + bytes([0xFF, cmd]) + size_bytes + payload + bytes([crc])


class FakeSerial:
    """Serial stand-in that replies with whatever the test queues up."""

    def __init__(self, replies=b""):
        self.rx = bytearray(replies)
        self.tx = bytearray()
        self.is_open = True

    @property
    def in_waiting(self):
        return len(self.rx)

    def read(self, n=1):
        n = min(n, len(self.rx))
        out, self.rx = bytes(self.rx[:n]), self.rx[n:]
        return out

    def write(self, data):
        self.tx += data
        return len(data)

    def flush(self):
        pass

    def reset_input_buffer(self):
        pass

    def reset_output_buffer(self):
        pass

    def close(self):
        self.is_open = False


def make_msp(replies=b"", timeout=0.2):
    m = MSP.__new__(MSP)
    m.port = "fake"
    m.timeout = timeout
    m.ser = FakeSerial(replies)
    import threading
    m._lock = threading.RLock()
    m._rx = b""
    m.crc_errors = m.timeouts = m.tx_frames = m.rx_frames = 0
    return m


class TestFraming(unittest.TestCase):
    def test_request_returns_payload(self):
        m = make_msp(frame(108, struct.pack("<hhh", 12, -34, 180)))
        self.assertEqual(struct.unpack("<hhh", m.request(108)), (12, -34, 180))

    def test_outgoing_frame_is_well_formed(self):
        m = make_msp(frame(200))
        m.request(200, b"\x01\x02")
        sent = bytes(m.ser.tx)
        self.assertTrue(sent.startswith(b"$M<"))
        self.assertEqual(sent[3], 2)          # size
        self.assertEqual(sent[4], 200)        # cmd
        crc = sent[3] ^ sent[4] ^ 0x01 ^ 0x02
        self.assertEqual(sent[-1], crc)

    def test_skips_replies_for_other_commands(self):
        m = make_msp(frame(101, b"\x01") + frame(108, struct.pack("<hhh", 1, 2, 3)))
        self.assertEqual(len(m.request(108)), 6)

    def test_bad_crc_is_counted_and_skipped(self):
        m = make_msp(frame(108, b"\x01\x02", corrupt_crc=True)
                     + frame(108, struct.pack("<hhh", 7, 8, 9)))
        self.assertEqual(struct.unpack("<hhh", m.request(108)), (7, 8, 9))
        self.assertEqual(m.crc_errors, 1)

    def test_leading_garbage_is_resynced(self):
        m = make_msp(b"junk\x00\xff$M" + frame(108, struct.pack("<hhh", 4, 5, 6)))
        self.assertEqual(struct.unpack("<hhh", m.request(108)), (4, 5, 6))

    def test_jumbo_frame(self):
        payload = bytes(range(256)) * 2
        m = make_msp(jumbo(119, payload))
        self.assertEqual(m.request(119), payload)

    def test_error_frame_raises(self):
        m = make_msp(frame(200, direction=b"!"))
        with self.assertRaises(MSPError):
            m.request(200)

    def test_timeout_raises_and_is_counted(self):
        m = make_msp(b"")
        with self.assertRaises(MSPTimeout):
            m.request(108)
        self.assertEqual(m.timeouts, 1)

    def test_split_frame_across_reads(self):
        f = frame(108, struct.pack("<hhh", 1, 2, 3))
        m = make_msp(f)
        # force byte-at-a-time delivery
        m.ser.read = lambda n=1, _s=m.ser: bytes([_s.rx.pop(0)]) if _s.rx else b""
        self.assertEqual(struct.unpack("<hhh", m.request(108)), (1, 2, 3))


class TestPortResolution(unittest.TestCase):
    def test_explicit_path_passes_through(self):
        self.assertEqual(msp_mod.resolve_port("/dev/ttyAMA0"), "/dev/ttyAMA0")

    def test_refuses_to_guess_between_two_boards(self):
        real = msp_mod.glob.glob
        msp_mod.glob.glob = lambda p: ["/dev/serial/by-id/a-if00",
                                       "/dev/serial/by-id/b-if00"]
        try:
            with self.assertRaises(MSPError):
                msp_mod.resolve_port(None)
        finally:
            msp_mod.glob.glob = real

    def test_clear_error_when_absent(self):
        real = msp_mod.glob.glob
        msp_mod.glob.glob = lambda p: []
        try:
            with self.assertRaises(MSPError):
                msp_mod.resolve_port(None)
        finally:
            msp_mod.glob.glob = real


if __name__ == "__main__":
    unittest.main()
