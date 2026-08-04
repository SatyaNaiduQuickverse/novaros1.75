"""MSP v1 client for Betaflight.

Framing: ``$M<`` request / ``$M>`` reply / ``$M!`` error, XOR checksum over
(size, cmd, payload). Jumbo frames (size byte 0xFF, real u16 size following the
command byte) carry payloads over 255 bytes — BOXNAMES needs them.

Every transaction is serialised on an internal lock, so an RC-stream thread and
a telemetry thread can share one port without interleaving their frames. The
receive buffer is drained before each request: MSP is strict request/response,
so anything already in the pipe is stale by definition.

Derived from the link layer verified on this Pi in ~/zerodrag/zerodrag_control.py
(SpeedyBee F7 V3, Betaflight 26.6.1, 0 CRC errors at 100 Hz).
"""

from __future__ import annotations

import glob
import struct
import threading
import time

import serial

# ---------------------------------------------------------------- command ids

MSP_API_VERSION = 1
MSP_FC_VARIANT = 2
MSP_FC_VERSION = 3
MSP_BOARD_INFO = 4
MSP_STATUS = 101
MSP_RAW_IMU = 102
MSP_MOTOR = 104
MSP_RC = 105
MSP_ATTITUDE = 108
MSP_MODE_RANGES = 34
MSP_BOXNAMES = 116
MSP_BOXIDS = 119
MSP_SET_RAW_RC = 200

# Permanent box ids — stable across firmware builds, unlike the bit positions
# inside MSP_STATUS's mode flags, which follow this build's box list order.
BOX_ARM_PERMANENT_ID = 0
BOX_MSP_OVERRIDE_PERMANENT_ID = 50


class MSPError(Exception):
    """Any MSP-layer failure: timeout, error frame, or port loss."""


class MSPTimeout(MSPError):
    pass


def resolve_port(hint: str | None = None) -> str:
    """Return a stable /dev/serial/by-id path for the flight controller.

    Never use raw /dev/ttyACM numbering: an ESP32 on this Pi also enumerates as
    a USB CDC device and can claim ttyACM0 depending on boot order.
    """
    if hint and not hint.startswith("auto"):
        return hint
    matches = sorted(glob.glob("/dev/serial/by-id/*Betaflight*-if00"))
    if not matches:
        raise MSPError(
            "no Betaflight FC found under /dev/serial/by-id/ — check the USB "
            "cable and that the FC is powered"
        )
    if len(matches) > 1 and hint is None:
        raise MSPError(
            "multiple Betaflight boards present, refusing to guess:\n  "
            + "\n  ".join(matches)
            + "\nset fc.port in config/vehicle.yaml to the one you mean"
        )
    return matches[0]


class MSP:
    """Thread-safe MSP v1 client."""

    def __init__(self, port: str, baud: int = 115200, timeout: float = 1.0):
        self.port = port
        self.timeout = timeout
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self._lock = threading.RLock()
        self._rx = b""
        # counters worth watching during bring-up
        self.crc_errors = 0
        self.timeouts = 0
        self.tx_frames = 0
        self.rx_frames = 0
        time.sleep(0.2)  # USB VCP settles; FC may still be emitting boot chatter
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    # ------------------------------------------------------------- framing

    @staticmethod
    def _crc(size: int, cmd: int, payload: bytes) -> int:
        c = size ^ cmd
        for b in payload:
            c ^= b
        return c

    def _encode(self, cmd: int, payload: bytes = b"") -> bytes:
        size = len(payload)
        if size > 254:
            raise MSPError("outgoing jumbo frames are not needed here")
        return b"$M<" + bytes([size, cmd]) + payload + bytes([self._crc(size, cmd, payload)])

    def _take_frame(self):
        """Pop one complete frame from the rx buffer.

        Returns (cmd, payload, is_error) or None when more bytes are needed.
        """
        while True:
            candidates = [p for p in (self._rx.find(b"$M>"), self._rx.find(b"$M!")) if p >= 0]
            if not candidates:
                # keep a possible split header, drop the rest
                if len(self._rx) > 2:
                    self._rx = self._rx[-2:]
                return None
            i = min(candidates)
            if i:
                self._rx = self._rx[i:]
                continue
            is_error = self._rx[1:3] == b"M!"
            if len(self._rx) < 5:
                return None
            size, cmd = self._rx[3], self._rx[4]
            if size == 0xFF:  # jumbo
                if len(self._rx) < 7:
                    return None
                jsize = struct.unpack("<H", self._rx[5:7])[0]
                end = 7 + jsize + 1
                if len(self._rx) < end:
                    return None
                payload = self._rx[7:end - 1]
                calc = self._crc(size, cmd, self._rx[5:7] + payload)
            else:
                end = 5 + size + 1
                if len(self._rx) < end:
                    return None
                payload = self._rx[5:end - 1]
                calc = self._crc(size, cmd, payload)
            got = self._rx[end - 1]
            self._rx = self._rx[end:]
            if got != calc:
                self.crc_errors += 1
                continue
            self.rx_frames += 1
            return cmd, payload, is_error

    # ------------------------------------------------------------ transport

    def _write(self, data: bytes) -> None:
        try:
            self.ser.write(data)
            self.ser.flush()
        except (serial.SerialException, OSError) as e:
            raise MSPError(f"serial write failed: {e}") from e
        self.tx_frames += 1

    def _read_some(self) -> None:
        try:
            self._rx += self.ser.read(self.ser.in_waiting or 1)
        except (serial.SerialException, OSError) as e:
            raise MSPError(f"serial read failed: {e}") from e

    # ---------------------------------------------------------- public API

    def send(self, cmd: int, payload: bytes = b"") -> None:
        """Fire a command without waiting for the reply."""
        with self._lock:
            self._write(self._encode(cmd, payload))

    def request(self, cmd: int, payload: bytes = b"", timeout: float | None = None) -> bytes:
        """Send a command and return the payload of its reply.

        Blocks up to ``timeout`` seconds (default: the port's timeout). Replies
        for other commands that arrive first are discarded.
        """
        deadline = time.time() + (self.timeout if timeout is None else timeout)
        with self._lock:
            self._rx = b""
            try:
                self.ser.reset_input_buffer()
            except (serial.SerialException, OSError):
                pass
            self._write(self._encode(cmd, payload))
            while time.time() < deadline:
                self._read_some()
                while True:
                    frame = self._take_frame()
                    if frame is None:
                        break
                    rcmd, rpayload, is_error = frame
                    if is_error and rcmd == cmd:
                        raise MSPError(f"FC returned an error frame for cmd {cmd}")
                    if rcmd == cmd:
                        return rpayload
        self.timeouts += 1
        raise MSPTimeout(f"no reply for cmd {cmd} within {self.timeout:.2f}s")

    def close(self) -> None:
        with self._lock:
            try:
                self.ser.close()
            except Exception:
                pass

    # ------------------------------------------------------- identification

    def identify(self) -> dict:
        api = self.request(MSP_API_VERSION)
        variant = self.request(MSP_FC_VARIANT).decode(errors="replace")
        ver = self.request(MSP_FC_VERSION)
        return {
            "variant": variant,
            "firmware": f"{ver[0]}.{ver[1]}.{ver[2]}",
            "msp_api": f"{api[1]}.{api[2]}",
            "port": self.port,
        }

    def box_ids(self) -> list[int]:
        """Permanent box ids in this build's flag-bit order."""
        return list(self.request(MSP_BOXIDS))
