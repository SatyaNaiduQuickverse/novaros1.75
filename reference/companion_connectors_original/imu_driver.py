"""imu_driver.py — IMU + attitude provider. TEMPLATE: fill in the real sensor read.

Contract the rest of the stack expects:
    imu.get_state() -> (q, w, accel_body)
      q          attitude quaternion, Hamilton (w,x,y,z)
      w          body angular rate, rad/s
      accel_body body specific-force accel, m/s^2, FRD frame (x fwd, y right, z down)

Two modes:
  RealIMU(fake=True)   -> returns a static level pose. Use this to bring up the rest
                          of the stack BEFORE the sensor is wired. No hardware needed.
  RealIMU(fake=False)  -> reads your MPU9250 (or similar) over SPI/I2C and runs an
                          AHRS. Fill in the two TODOs below.
"""
import time, threading
import numpy as np


class RealIMU:
    def __init__(self, hz=200.0, fake=False):
        self.dt = 1.0 / hz
        self.fake = fake
        self._q = np.array([1.0, 0, 0, 0])
        self._w = np.zeros(3)
        self._a = np.array([0.0, 0, -9.80665])
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._th = None
        if not fake:
            self._open()

    # ---- fill these two in for real hardware --------------------------------
    def _open(self):
        # TODO: open SPI/I2C to the IMU (e.g. spidev / smbus2), configure ranges,
        # zero the gyro at rest. Raise if the device is not found.
        raise NotImplementedError("wire your MPU9250 SPI/I2C init here (or use fake=True)")

    def _read_raw(self):
        # TODO: return (gyro_rad_s[3], accel_ms2[3], mag[3] or None) in the SENSOR frame,
        # already scaled to physical units. Apply the IMU->body mounting rotation here so
        # what you return is in the airframe FRD frame.
        raise NotImplementedError("wire your raw IMU read here")
    # -------------------------------------------------------------------------

    def _ahrs_update(self, gyro, accel, mag, dt):
        # TODO: run a standard AHRS (Madgwick/Mahony) to fuse gyro+accel(+mag) into q.
        # Placeholder: integrate gyro only (drifts) — REPLACE with a real AHRS.
        from math_utils import q_mult, q_normalize
        wq = np.array([0.0, *gyro])
        self._q = q_normalize(self._q + 0.5 * q_mult(self._q, wq) * dt)
        return self._q

    def _loop(self):
        while not self._stop.is_set():
            t0 = time.time()
            gyro, accel, mag = self._read_raw()
            q = self._ahrs_update(gyro, accel, mag, self.dt)
            with self._lock:
                self._q, self._w, self._a = q, np.asarray(gyro, float), np.asarray(accel, float)
            slp = self.dt - (time.time() - t0)
            if slp > 0:
                time.sleep(slp)

    def start(self):
        if not self.fake:
            self._th = threading.Thread(target=self._loop, daemon=True)
            self._th.start()
        return self

    def get_state(self):
        with self._lock:
            return self._q.copy(), self._w.copy(), self._a.copy()

    def stop(self):
        self._stop.set()
