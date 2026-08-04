"""Attitude and IMU providers.

Contract the rest of the stack consumes:

    imu.get_state() -> (q, w, accel_body)
      q           attitude quaternion, Hamilton (w,x,y,z), body FRD -> world NED
      w           body angular rate, rad/s, FRD
      accel_body  specific-force acceleration, m/s^2, FRD (x fwd, y right, z down)

Three implementations:

  :class:`FCIMU`    reads the flight controller's own fused attitude and its
                    gyro/accel over MSP. Betaflight is already running a tuned
                    AHRS on a calibrated sensor, so on this airframe this is
                    the one to use — no extra hardware, no filter to write.
  :class:`FakeIMU`  a static level pose, for bringing up the rest of the stack
                    before anything is wired.
  :class:`RealIMU`  template for a *dedicated* companion-side IMU (MPU9250 or
                    similar) when you want state independent of the FC. Two
                    TODOs plus a real AHRS.

``RealIMU(fake=True)`` still works and returns a :class:`FakeIMU`, so the
original harness invocations keep running.
"""

from __future__ import annotations

import logging
import threading
import time

import numpy as np

from .math_utils import euler_to_q, q_mult, q_normalize

log = logging.getLogger("companion.imu")

LEVEL_ACCEL_FRD = np.array([0.0, 0.0, -9.80665])


class _Base:
    def start(self):
        return self

    def stop(self):
        pass

    def get_state(self):
        raise NotImplementedError


class FakeIMU(_Base):
    """Static level pose. No hardware, no thread."""

    def __init__(self, **_):
        self._q = np.array([1.0, 0.0, 0.0, 0.0])
        self._w = np.zeros(3)
        self._a = LEVEL_ACCEL_FRD.copy()

    def get_state(self):
        return self._q.copy(), self._w.copy(), self._a.copy()


class FCIMU(_Base):
    """Attitude and IMU from the flight controller over MSP.

    Uses whatever the FC's telemetry thread has cached, so polling this is
    free — it never adds MSP traffic of its own. Start the link's telemetry
    thread first (``fc.start_telemetry()``) or every call does a blocking read.
    """

    def __init__(self, fc, stale_after_s: float = 0.5):
        self.fc = fc
        self.stale_after_s = stale_after_s
        self._last_good = (np.array([1.0, 0.0, 0.0, 0.0]), np.zeros(3),
                           LEVEL_ACCEL_FRD.copy())
        self._last_t = 0.0

    def get_state(self):
        att = self.fc.read_attitude()
        imu = self.fc.read_imu()
        if att is None or imu is None:
            # Hold the last good sample; the logger records the staleness.
            return tuple(x.copy() for x in self._last_good)
        q = euler_to_q(att["roll"], att["pitch"], att["yaw"])
        accel, gyro = imu
        self._last_good = (q, np.asarray(gyro, float), np.asarray(accel, float))
        self._last_t = time.time()
        return tuple(x.copy() for x in self._last_good)

    def stale(self) -> bool:
        return (time.time() - self._last_t) > self.stale_after_s


class RealIMU(_Base):
    """Dedicated companion-side IMU. TEMPLATE — two TODOs and an AHRS.

    Only needed if you want an attitude source independent of the flight
    controller. Otherwise use :class:`FCIMU`.
    """

    def __init__(self, hz: float = 200.0, fake: bool = False):
        if fake:
            raise TypeError("use FakeIMU() — see make_imu()")
        self.dt = 1.0 / hz
        self._q = np.array([1.0, 0.0, 0.0, 0.0])
        self._w = np.zeros(3)
        self._a = LEVEL_ACCEL_FRD.copy()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._th = None
        self._open()

    # ---- fill these in for real hardware ------------------------------------
    def _open(self):
        # TODO: open SPI/I2C to the IMU (spidev / smbus2), configure ranges,
        # zero the gyro at rest. Raise if the device is not found.
        raise NotImplementedError(
            "wire your MPU9250 SPI/I2C init here, or use FCIMU / FakeIMU"
        )

    def _read_raw(self):
        # TODO: return (gyro_rad_s[3], accel_ms2[3], mag[3] or None), already
        # scaled to physical units AND rotated by the IMU->body mounting
        # rotation, so what you return is in airframe FRD.
        raise NotImplementedError("wire your raw IMU read here")
    # -------------------------------------------------------------------------

    def _ahrs_update(self, gyro, accel, mag, dt):
        # TODO: run a real AHRS (Madgwick/Mahony) fusing gyro+accel(+mag).
        # Placeholder integrates gyro only and therefore drifts — REPLACE.
        wq = np.array([0.0, *gyro])
        self._q = q_normalize(self._q + 0.5 * q_mult(self._q, wq) * dt)
        return self._q

    def _loop(self):
        while not self._stop.is_set():
            t0 = time.time()
            gyro, accel, mag = self._read_raw()
            q = self._ahrs_update(gyro, accel, mag, self.dt)
            with self._lock:
                self._q = q
                self._w = np.asarray(gyro, float)
                self._a = np.asarray(accel, float)
            slp = self.dt - (time.time() - t0)
            if slp > 0:
                time.sleep(slp)

    def start(self):
        self._th = threading.Thread(target=self._loop, name="imu", daemon=True)
        self._th.start()
        return self

    def get_state(self):
        with self._lock:
            return self._q.copy(), self._w.copy(), self._a.copy()

    def stop(self):
        self._stop.set()


def make_imu(source: str = "fc", fc=None, cfg=None, **kw) -> _Base:
    """Build the configured attitude source.

    source:
      "esp32"  dedicated MPU6500 on an ESP32-C6 over USB — the flight article
      "fc"     attitude from the flight controller over MSP — bring-up only,
               costs one MSP transaction per sample from a budget that already
               caps the control rate
      "fake"   static level pose, no hardware
      "real"   template for a Pi-attached IMU (unimplemented)
    """
    if source == "fake":
        return FakeIMU()
    if source == "esp32":
        from .imu_esp32 import ESP32IMU
        cal = getattr(cfg, "imu32", None) if cfg is not None else None
        return ESP32IMU(port=(cal.port or None) if cal else None, cal=cal,
                        kp=getattr(cal, "kp", 2.0), ki=getattr(cal, "ki", 0.05),
                        **kw)
    if source == "fc":
        if fc is None:
            raise ValueError("source='fc' needs a connected FCLink")
        return FCIMU(fc, **kw)
    if source == "real":
        return RealIMU(**kw)
    raise ValueError(f"unknown IMU source {source!r}")
