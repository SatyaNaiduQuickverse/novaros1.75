"""Tracker bounding box -> world-frame bearing.

The pixel -> body-ray -> world math is complete. What you supply is:

  A. a bbox source — ``get_bbox_fn() -> ([x, y, w, h], conf)`` in pixels, or
     ``(None, 0.0)`` when the tracker is not confident. On this Pi that is the
     chase2 detector/tracker stack; see :func:`http_bbox_source` for the usual
     wiring.
  B. calibration — intrinsics, distortion, and the camera-in-body basis. These
     live in ``config/vehicle.yaml`` under ``camera``. Until they are measured
     every bearing carries a silent, constant angular bias.

    va = VisionAdapter(cfg.camera, get_bbox_fn=tracker, get_attitude_fn=imu_q)
    b = va.bearing(time.time())     # -> Bearing | None
"""

from __future__ import annotations

import logging

import numpy as np

from .config import CameraCal
from .math_utils import q_to_R
from .vision_interface import Bearing, VisionSource

log = logging.getLogger("companion.vision")


class VisionAdapter(VisionSource):
    def __init__(self, cal: CameraCal | None = None, get_bbox_fn=None,
                 get_attitude_fn=None, fake: bool = False):
        self.cal = cal or CameraCal()
        self.get_bbox = get_bbox_fn
        self.get_attitude = get_attitude_fn
        self.fake = fake
        self._undistort = _make_undistorter(self.cal)
        self.cam_fwd = np.asarray(self.cal.cam_fwd, float)
        self.cam_right = np.asarray(self.cal.cam_right, float)
        self.cam_up = np.asarray(self.cal.cam_up, float)
        if not fake and get_bbox_fn is None:
            log.warning("no bbox source wired — bearing() will always return None")

    def _attitude(self):
        if self.get_attitude is None:
            return np.array([1.0, 0.0, 0.0, 0.0])
        return np.asarray(self.get_attitude(), float)

    def bearing(self, t) -> Bearing | None:
        if self.fake:
            # Object parked straight down the boresight, so the rest of the
            # stack sees a plausible, attitude-consistent measurement.
            u = q_to_R(self._attitude()) @ self.cam_fwd
            return Bearing(u, range_m=None, conf=1.0, t=t)

        if self.get_bbox is None:
            return None
        bbox, conf = self.get_bbox()
        if bbox is None:
            return None
        x, y, w, h = bbox
        cx, cy = self._undistort(x + w / 2.0, y + h / 2.0)

        # pixel -> unit ray in the body frame
        ray = (self.cam_right * ((cx - self.cal.cx) / self.cal.fx)
               + self.cam_up * (-(cy - self.cal.cy) / self.cal.fy)
               + self.cam_fwd)
        ray /= (np.linalg.norm(ray) + 1e-9)

        # body -> world using the attitude time-matched to this frame
        u_world = q_to_R(self._attitude()) @ ray

        # Size-based range hint. Only meaningful once the box is well resolved;
        # below min_box_px a one-pixel error swings the estimate wildly.
        rng = None
        if w > self.cal.min_box_px:
            rng = 1.6 * self.cal.object_size_m * self.cal.fx / max(w, 1e-6)
        return Bearing(u_world, range_m=rng, conf=float(conf), t=t)


def _make_undistorter(cal: CameraCal):
    """Return a (px, py) -> (px, py) undistort function.

    Uses OpenCV when distortion coefficients are configured; otherwise identity.
    """
    if not cal.dist_coeffs:
        return lambda px, py: (px, py)
    try:
        import cv2
    except ImportError:
        log.warning("dist_coeffs configured but OpenCV is missing — not undistorting")
        return lambda px, py: (px, py)
    K = np.array([[cal.fx, 0, cal.cx], [0, cal.fy, cal.cy], [0, 0, 1]], float)
    D = np.array(cal.dist_coeffs, float).reshape(1, -1)

    def undistort(px, py):
        pt = np.array([[[float(px), float(py)]]], dtype=np.float64)
        out = cv2.undistortPoints(pt, K, D, P=K)
        return float(out[0, 0, 0]), float(out[0, 0, 1])

    return undistort


def http_bbox_source(url: str, timeout: float = 0.05, min_conf: float = 0.4):
    """Poll a tracker's HTTP telemetry endpoint for the current box.

    Expects JSON like ``{"bbox": [x, y, w, h], "conf": 0.87}``. Returns
    ``(None, 0.0)`` on any error or when confidence is below ``min_conf`` —
    a missing measurement is always safer than a wrong one.
    """
    import json
    import urllib.request

    def get_bbox():
        try:
            with urllib.request.urlopen(url, timeout=timeout) as r:
                d = json.loads(r.read().decode())
            bbox, conf = d.get("bbox"), float(d.get("conf", 0.0))
            if not bbox or conf < min_conf:
                return None, 0.0
            return list(bbox), conf
        except Exception:
            return None, 0.0

    return get_bbox
