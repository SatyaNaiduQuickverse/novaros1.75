"""Dedicated companion IMU: MPU6500 on an ESP32-C6, read over USB CDC.

This is the attitude source for the flight article. ``FCIMU`` (attitude from
the flight controller over MSP) is a bring-up convenience and has two problems
that matter in flight: the FC's attitude is quantised to 0.1 deg and arrives
late, and every sample costs an MSP transaction from a budget that is already
the binding constraint on control rate. This path costs the FC nothing.

    Pi  <--USB CDC--  ESP32-C6  <--I2C 400kHz--  MPU6500
                      SCL GPIO19 / SDA GPIO20

The ESP32 ships raw counts (see esp32/main.py); scaling, bias removal, the
axis map and the attitude filter all live here, so they can be changed without
reflashing.

⚠️ **Read docs/ESP32_BRIDGE_FAULTS.md before changing anything here.** Two
measured faults drive most of the odd-looking code below — a bridge that sits
enumerated and permanently silent, and an accelerometer whose reported gravity
magnitude depends on which axis it lands on. Both present as "nothing is
wrong" and both yield a plausible, incorrect attitude.

**No magnetometer.** The MPU6500 is a 6-axis part, so roll and pitch are
observable from gravity but yaw has no absolute reference and will drift. The
filter holds yaw against the gyro only. Anything needing true heading must get
it elsewhere — the FC's compass, or by initialising yaw from the FC once at
engage.
"""

from __future__ import annotations

import glob
import logging
import math
import struct
import threading
import time

import numpy as np

from .math_utils import q_normalize, q_to_R

log = logging.getLogger("companion.imu32")

G = 9.80665
SYNC = b"\xA5\x5A"
FRAME_LEN = 18

# Must match the ranges configured in esp32/main.py.
ACCEL_FS_G = 16.0          # +/-16 g   -> 2048 LSB/g nominal
GYRO_FS_DPS = 2000.0       # +/-2000 dps -> 16.4 LSB/dps nominal
ACCEL_LSB_PER_G = 32768.0 / ACCEL_FS_G
GYRO_LSB_PER_DPS = 32768.0 / GYRO_FS_DPS


def find_esp32_port() -> str:
    ports = sorted(glob.glob("/dev/serial/by-id/*Espressif*"))
    if not ports:
        raise FileNotFoundError(
            "no ESP32 found under /dev/serial/by-id/ — is the IMU bridge "
            "plugged in and flashed?")
    return ports[0]


# The bridge waits this long after boot for a host to announce itself before
# streaming anyway (esp32/main.py STARTUP_DELAY_S). The Pi must not call that
# silence a wedge, so recovery is suppressed until it has elapsed plus a margin
# for ROM + MicroPython boot. Firmware currently on the device still uses the
# old 2 s, so this is generous rather than tight — being late to declare a wedge
# costs seconds, being early costs a reset loop.
BOOT_GRACE_S = 14.0
# Any byte that is not Ctrl-C tells the bridge a host is listening, so it can
# stop waiting out STARTUP_DELAY_S and start streaming immediately.
WAKE_BYTE = b"\x01"


def assert_dtr(ser) -> None:
    """Raise DTR. The bridge sends NOTHING while it is low.

    MEASURED 2026-08-04, six for six across 115200/921600 on a freshly-opened
    handle: DTR high gives 3618 B/s, DTR low gives 130 B/s — the tail of what
    was already buffered, then silence. Linux deasserts DTR whenever no
    process holds the tty open, so an idle port looks to the bridge like an
    absent computer, which is the state the companion boots into.

    Scope, honestly: this did NOT reproduce once the reader already held the
    port open and had reset the board — dropping DTR there left the stream
    running at 200 Hz. So treat it as "an unheld port means a silent bridge",
    not as a live kill switch. The consequence is the same either way: hold
    the port, and keep DTR asserted while holding it.
    """
    try:
        ser.dtr = True
    except Exception:
        pass


def hardware_reset(port: str, settle_s: float = 0.15) -> None:
    """Reboot the ESP32 over the USB-Serial-JTAG control lines.

    MEASURED 2026-08-04: the bridge can wedge with the USB device still
    enumerated and streaming nothing at all — opening and reading the port does
    not revive it, only a chip reset does. It happens when nothing drains the
    CDC endpoint in the first seconds after boot, which is exactly what a
    companion reboot looks like: the ESP32 came up 4 s before the Pi's cdc_acm
    driver attached, blocked on its first writes and never recovered. Nineteen
    minutes of silent, healthy-looking "no attitude" is the failure mode this
    exists to end.

    On the C6 the USB-Serial-JTAG peripheral is in hardware, so it enumerates
    whether or not the firmware is alive — presence on /dev/serial/by-id proves
    nothing. DTR drives GPIO9 (boot select) and RTS drives EN (reset); DTR is
    held FALSE throughout so the part can only ever come up in normal flash
    boot, never the ROM download mode.
    """
    import serial
    with serial.Serial(port, 115200, timeout=0.1, write_timeout=0.3) as s:
        pulse_reset(s)


def pulse_reset(ser, settle_s: float = 0.15) -> None:
    """Reset the chip using an ALREADY-OPEN handle, then wake it.

    Taking the open handle matters, and is the whole fix for the boot race: the
    kernel's cdc_acm driver only submits read URBs while the tty is open, so an
    ESP32 that boots before anything opens the port is streaming into a closed
    door — which is what wedged it. Reset it while we are already holding the
    port and it can never lose that race, no matter how slow the companion is
    to come up.
    """
    ser.dtr = False         # GPIO9 high: normal flash boot, never download mode
    ser.rts = True          # assert EN low
    time.sleep(settle_s)
    ser.rts = False         # release
    ser.dtr = True          # host present
    time.sleep(0.6)         # ROM + MicroPython boot
    try:
        # A bounded write is mandatory, not tidiness: the firmware on the
        # device cannot drain its OUT endpoint, so an unbounded write here
        # blocks the caller forever — start() included.
        ser.write_timeout = 0.3
        ser.write(WAKE_BYTE)    # "a host is listening" — skips STARTUP_DELAY_S
    except Exception:
        # The build on the device today cannot drain its OUT endpoint while
        # streaming, so this write times out and is meant to. It is an
        # optimisation for the next firmware, never a requirement.
        pass
    try:
        ser.reset_input_buffer()
    except Exception:
        pass


class Mahony:
    """Complementary attitude filter, gyro corrected toward measured gravity.

    Chosen over Madgwick for having one obvious knob per behaviour: ``kp`` sets
    how hard gravity pulls the estimate straight, ``ki`` removes residual gyro
    bias. With no magnetometer there is no yaw correction term at all.
    """

    def __init__(self, kp: float = 2.0, ki: float = 0.05):
        self.kp, self.ki = kp, ki
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self._ei = np.zeros(3)

    def update(self, gyro, accel, dt):
        """gyro rad/s and accel m/s^2, both body FRD. Returns the quaternion."""
        w = np.asarray(gyro, float).copy()
        a = np.asarray(accel, float)
        n = np.linalg.norm(a)
        if n > 1e-6:
            a_hat = a / n
            # Specific force at rest points UP, so in body FRD it is -z_world
            # expressed in body: minus the third row of R (world->body).
            R = q_to_R(self.q)
            v = -R[2, :]
            e = np.cross(a_hat, v)
            if self.ki:
                self._ei += e * dt
            w = w + self.kp * e + self.ki * self._ei

        wq = np.array([0.0, w[0], w[1], w[2]])
        qw, qx, qy, qz = self.q
        dq = 0.5 * np.array([
            -qx * wq[1] - qy * wq[2] - qz * wq[3],
            qw * wq[1] + qy * wq[3] - qz * wq[2],
            qw * wq[2] - qx * wq[3] + qz * wq[1],
            qw * wq[3] + qx * wq[2] - qy * wq[1],
        ])
        self.q = q_normalize(self.q + dq * dt)
        return self.q


class ESP32IMU:
    """Reads the ESP32 bridge and presents the standard IMU contract.

    ``get_state() -> (q, w, accel_body)`` — see docs/COMMAND_MODULE_INTERFACE.md.
    """

    def __init__(self, port: str | None = None, baud: int = 921600,
                 cal=None, kp: float = 2.0, ki: float = 0.05,
                 stale_after_s: float = 0.1, recover_after_s: float = 3.0):
        self.port = port
        self.baud = baud
        self.cal = cal
        self.stale_after_s = stale_after_s
        # Seconds of total silence before the reader reboots the bridge; 0
        # disables it. See hardware_reset() for why this is not optional in
        # practice. Well above stale_after_s: stale() is the control loop's
        # "do not trust this sample", this is "the bridge is gone".
        self.recover_after_s = recover_after_s
        self.ahrs = Mahony(kp, ki)

        self.gyro_bias = np.zeros(3)
        self.accel_per_g = ACCEL_LSB_PER_G
        self.accel_offset = np.zeros(3)
        self.axis_map = ((0, 1.0), (1, 1.0), (2, 1.0))   # sensor -> body FRD
        if cal is not None:
            self.gyro_bias = np.asarray(getattr(cal, "gyro_bias", [0, 0, 0]), float)
            self.accel_per_g = float(getattr(cal, "accel_per_g", ACCEL_LSB_PER_G))
            self.accel_offset = np.asarray(
                getattr(cal, "accel_offset", [0.0, 0.0, 0.0]), float)
            self.axis_map = tuple(getattr(cal, "axis_map", self.axis_map))
        # Counts per g, per sensor axis. MEASURED 2026-08-04: this part is not
        # well described by one number — at rest it reported |a| = 2400 counts
        # with gravity on sensor z and 2001 with gravity on sensor x, a 20%
        # disagreement between two orientations that both sat still to within
        # 4 counts. A scalar scale cannot represent that, and the residual
        # lands where it hurts most: the Mahony filter normalises the vector,
        # so a common scale error is invisible to attitude while a per-axis
        # offset tilts the gravity direction degree for degree.
        per_axis = tuple(getattr(cal, "accel_per_g_axis", ()) or ()) if cal else ()
        self.accel_scale = (np.asarray(per_axis, float) if len(per_axis) == 3
                            else np.full(3, self.accel_per_g))

        self._ser = None
        self._th = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._q = np.array([1.0, 0.0, 0.0, 0.0])
        self._w = np.zeros(3)
        self._a = np.array([0.0, 0.0, -G])
        self._last_t = 0.0
        # Raw counts as they arrived, before bias removal or the axis map —
        # calibration must work from these or it calibrates its own output.
        self._raw_gyro = np.zeros(3)
        self._raw_accel = np.zeros(3)
        # health counters — a dropped-sample rate is the first sign of a bad
        # cable or an overloaded bridge, and it is invisible without this.
        self.frames = 0
        self.drops = 0
        self.checksum_errors = 0
        self.resyncs = 0
        self.recoveries = 0
        self.nudges = 0
        self._last_seq = None
        self._last_rx = 0.0
        self._grace_until = 0.0

    # ------------------------------------------------------------ lifecycle

    def start(self, reset: bool = True) -> "ESP32IMU":
        """Open the port, reboot the bridge into it, and start reading.

        ``reset`` is on by default and is the fix for the boot race described
        in ``pulse_reset``: rebooting the bridge while we already hold the port
        open makes "companion started late" impossible by construction, rather
        than something the firmware has to be lucky about. It costs ~3 s at
        startup. Pass ``reset=False`` only to attach to a bridge that is
        already known to be streaming.
        """
        self.port = self.port or find_esp32_port()
        self._open()
        if reset:
            pulse_reset(self._ser)
            self._grace_until = time.monotonic() + BOOT_GRACE_S
        self._last_rx = time.monotonic()
        self._th = threading.Thread(target=self._loop, name="esp32-imu", daemon=True)
        self._th.start()
        if reset:
            self.wait_ready()
        return self

    def wait_ready(self, timeout_s: float = 8.0) -> bool:
        """Block until frames are arriving. False on timeout — never raises."""
        deadline = time.monotonic() + timeout_s
        n0 = self.frames
        while time.monotonic() < deadline:
            if self.frames > n0 + 5:
                return True
            time.sleep(0.05)
        return False

    def _open(self):
        import serial
        self._ser = serial.Serial(self.port, self.baud, timeout=0.05,
                                  write_timeout=0.3)
        assert_dtr(self._ser)
        time.sleep(0.05)
        self._ser.reset_input_buffer()

    def _nudge(self) -> bool:
        """Re-assert DTR and see if that alone brings the stream back.

        Tried before rebooting because it is nearly free and costs no outage:
        the bridge goes quiet whenever DTR drops (see assert_dtr), and that is
        a far more common cause of silence than a genuine hang. A reset costs
        ~3 s of no attitude; this costs 0.4 s and usually suffices.
        """
        if self._ser is None:
            return False
        try:
            self._ser.dtr = False
            time.sleep(0.05)
            self._ser.dtr = True
        except Exception:
            return False
        deadline = time.monotonic() + 0.4
        while time.monotonic() < deadline:
            try:
                if self._ser.in_waiting:
                    self.nudges += 1
                    log.info("ESP32 stream resumed on DTR re-assert (no reset)")
                    self._last_rx = time.monotonic()
                    return True
            except Exception:
                return False
            time.sleep(0.02)
        return False

    def _recover(self):
        """Revive the bridge: nudge DTR first, reboot only if that fails."""
        if self._nudge():
            return
        self.recoveries += 1
        log.warning("ESP32 bridge silent for %.1fs — resetting (recovery #%d)",
                    self.recover_after_s, self.recoveries)
        try:
            if self._ser:
                self._ser.close()
        except Exception:
            pass
        self._ser = None
        try:
            # Reopen FIRST, then reset through the open handle, so the bridge
            # boots into a port that is already being drained.
            self._open()
            pulse_reset(self._ser)
        except Exception as e:
            log.warning("ESP32 recovery failed: %s", e)
            time.sleep(1.0)
        self._grace_until = time.monotonic() + BOOT_GRACE_S
        # The bridge restarts its sequence counter, so forget the old one —
        # otherwise the discontinuity is charged to `drops` and a clean
        # recovery reads as 68 lost samples.
        self._last_seq = None
        # Restart the silence clock either way, so a port that stays dead
        # retries on the same cadence instead of spinning on resets.
        self._last_rx = time.monotonic()

    def stop(self):
        self._stop.set()
        if self._th:
            self._th.join(timeout=1.0)
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass

    # ---------------------------------------------------------------- read

    def _to_body(self, v):
        return np.array([sign * v[src] for src, sign in self.axis_map], float)

    def _loop(self):
        buf = bytearray()
        while not self._stop.is_set():
            if self._ser is None:
                self._recover()
                continue
            try:
                # Drain whatever is buffered rather than a fixed slice: a
                # fixed 256-byte read at this timeout caps throughput at very
                # nearly the 3600 B/s the bridge emits, so the kernel buffer
                # creeps upward and the stream stalls after tens of seconds.
                waiting = self._ser.in_waiting
                chunk = self._ser.read(waiting if waiting else 1)
            except Exception as e:
                log.warning("ESP32 serial read failed: %s", e)
                time.sleep(0.1)
                continue
            if chunk:
                buf += chunk
                self._last_rx = time.monotonic()
            elif (self.recover_after_s
                  and time.monotonic() > self._grace_until
                  and time.monotonic() - self._last_rx > self.recover_after_s):
                # Silence, not slowness: the bridge emits 3600 B/s or nothing.
                self._recover()
                buf.clear()
                continue
            while len(buf) >= FRAME_LEN:
                i = buf.find(SYNC)
                if i < 0:
                    # Nothing framed in here — keep only a possible split sync.
                    del buf[:-1]
                    break
                if i:
                    del buf[:i]
                    self.resyncs += 1
                    continue
                if len(buf) < FRAME_LEN:
                    break
                frame = bytes(buf[:FRAME_LEN])
                del buf[:FRAME_LEN]
                x = 0
                for b in frame[2:17]:
                    x ^= b
                if x != frame[17]:
                    self.checksum_errors += 1
                    continue
                self._on_frame(frame)

    def _on_frame(self, frame: bytes):
        seq = frame[2]
        ax, ay, az, temp, gx, gy, gz = struct.unpack_from("<7h", frame, 3)
        if self._last_seq is not None:
            gap = (seq - self._last_seq) & 0xFF
            if gap != 1:
                self.drops += gap - 1
        self._last_seq = seq
        self.frames += 1

        raw_a = np.array([ax, ay, az], float)
        raw_g = np.array([gx, gy, gz], float)
        self._raw_accel, self._raw_gyro = raw_a, raw_g
        # Offset and scale are applied in the SENSOR frame, before the axis
        # map — they are properties of the die, not of how it is bolted in.
        accel = self._to_body((raw_a - self.accel_offset) * (G / self.accel_scale))
        gyro = self._to_body((raw_g - self.gyro_bias)
                             * (math.pi / 180.0) / GYRO_LSB_PER_DPS)

        now = time.monotonic()
        dt = now - self._last_t if self._last_t else 1.0 / 200.0
        self._last_t = now
        dt = min(max(dt, 1e-4), 0.05)      # guard against scheduler hiccups
        q = self.ahrs.update(gyro, accel, dt)

        with self._lock:
            self._q, self._w, self._a = q.copy(), gyro, accel
        self.temp_c = temp / 333.87 + 21.0

    # --------------------------------------------------------------- output

    def get_state(self):
        with self._lock:
            return self._q.copy(), self._w.copy(), self._a.copy()

    def stale(self) -> bool:
        return (time.monotonic() - self._last_t) > self.stale_after_s

    def stats(self) -> dict:
        return {"frames": self.frames, "drops": self.drops,
                "checksum_errors": self.checksum_errors, "resyncs": self.resyncs,
                "recoveries": self.recoveries, "nudges": self.nudges,
                "stale": self.stale()}

    # ---------------------------------------------------------- calibration

    def calibrate(self, seconds: float = 4.0) -> dict:
        """Measure gyro bias and accelerometer scale at rest.

        The vehicle must be still. Works from raw counts, so it is independent
        of whatever bias and scale are currently configured.

        Accel scale needs no level surface: at rest the sensor measures exactly
        1 g in *some* direction, so the magnitude alone gives counts-per-g. The
        axis map does need level, and that is a separate step.
        """
        gyro, mag = [], []
        stop = time.monotonic() + seconds
        n0 = self.frames
        while time.monotonic() < stop:
            with self._lock:
                g, a = self._raw_gyro.copy(), self._raw_accel.copy()
            if a.any():
                gyro.append(g)
                mag.append(float(np.linalg.norm(a)))
            time.sleep(0.004)
        if len(gyro) < 10:
            raise RuntimeError("too few samples — is the bridge streaming?")
        bias = np.mean(gyro, axis=0)
        counts_per_g = float(np.mean(mag))
        spread = float(np.std(mag))
        moved = spread > 0.02 * counts_per_g
        return {
            "gyro_bias": [round(float(x), 1) for x in bias],
            "gyro_bias_dps": [round(float(x) / GYRO_LSB_PER_DPS, 2) for x in bias],
            "accel_per_g": round(counts_per_g, 1),
            "accel_spread": round(spread, 1),
            "samples": len(gyro),
            "frames": self.frames - n0,
            "moved_during_cal": moved,
        }
