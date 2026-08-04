"""vision_adapter.py — turn a tracker's pixel bounding box into a world-frame bearing.

TEMPLATE: wire your tracker source (section A) and set your calibration (section B).
The pixel -> body-ray -> world math is done for you (section C). Produces a
`Bearing` (from vision_interface) that the rest of the stack consumes.

    va = VisionAdapter(get_bbox_fn=<your tracker>, get_attitude_fn=imu_q, fake=False)
    b  = va.bearing(t)          # -> Bearing(u_world, range_m, conf, t) or None
"""
import numpy as np
from math_utils import q_to_R
from vision_interface import Bearing


class VisionAdapter:
    def __init__(self, get_bbox_fn=None, get_attitude_fn=None, fake=False,
                 W=1280, H=720):
        # ---- A. TRACKER SOURCE --------------------------------------------------
        # get_bbox_fn() must return (bbox, conf) where bbox=[x,y,w,h] in pixels, or (None,0).
        # e.g. poll your tracker's HTTP /telemetry, or call its Python API. Only return a
        # box when it is confident and the object is well resolved; else (None, 0).
        self.get_bbox = get_bbox_fn
        # get_attitude_fn() must return the CURRENT attitude quaternion (w,x,y,z),
        # time-matched to the frame (see SETUP_HARDWARE.txt on time-sync).
        self.get_attitude = get_attitude_fn
        self.fake = fake

        # ---- B. CALIBRATION (set these from your bench cal — SETUP_HARDWARE.txt §5) ----
        self.W, self.H = W, H
        self.fx = 1650.0 * (W / 1280.0)   # focal length in px  <-- REPLACE with your cal
        self.fy = self.fx                 # <-- REPLACE (fy from your cal)
        self.cx0 = W / 2.0                # principal point x    <-- REPLACE with your cal
        self.cy0 = H / 2.0                # principal point y    <-- REPLACE with your cal
        self.undistort = lambda px, py: (px, py)  # <-- REPLACE with your lens undistort
        # camera basis in the BODY frame (boresight = mounting alignment, §5).
        # default: boresight along body +x (forward), image-right = body +y, image-up = body -z.
        self.cam_fwd   = np.array([1.0, 0.0, 0.0])
        self.cam_right = np.array([0.0, 1.0, 0.0])
        self.cam_up    = np.array([0.0, 0.0, -1.0])
        self.object_size_m = 0.4          # object's widest dimension, for the size-range hint

    # ---- C. pixel box -> world bearing (done for you) --------------------------
    def bearing(self, t):
        if self.fake:
            q = self.get_attitude() if self.get_attitude else np.array([1.0, 0, 0, 0])
            u = q_to_R(q) @ self.cam_fwd            # object straight ahead of the boresight
            return Bearing(u / (np.linalg.norm(u) + 1e-9), range_m=None, conf=1.0, t=t)

        bbox, conf = self.get_bbox()
        if bbox is None:
            return None
        x, y, w, h = bbox
        cx, cy = self.undistort(x + w / 2.0, y + h / 2.0)
        # pixel -> normalized body-frame ray
        ray = (self.cam_right * ((cx - self.cx0) / self.fx)
               + self.cam_up * (-(cy - self.cy0) / self.fy)
               + self.cam_fwd)
        ray /= (np.linalg.norm(ray) + 1e-9)
        # body -> world using the CURRENT attitude estimate
        q = self.get_attitude()
        u_world = q_to_R(q) @ ray
        u_world /= (np.linalg.norm(u_world) + 1e-9)
        # size-based range hint (only trust when the box is clearly resolved)
        rng = 1.6 * self.object_size_m * self.fx / max(w, 1e-6) if w > 26 else None
        return Bearing(u_world, range_m=rng, conf=float(conf), t=t)
