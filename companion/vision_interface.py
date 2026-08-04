"""The vision contract: a world-frame unit bearing to the tracked object."""

from __future__ import annotations

import numpy as np


class Bearing:
    """A line-of-sight measurement at time ``t``.

    u_world  unit vector from the vehicle toward the object, world NED
    range_m  optional range estimate (size-based hint), or None when unreliable
    conf     tracker confidence, 0..1
    t        timestamp of the FRAME the measurement came from, not of its arrival
    """

    __slots__ = ("u_world", "range_m", "conf", "t")

    def __init__(self, u_world, range_m=None, conf=1.0, t=0.0):
        u = np.asarray(u_world, float)
        self.u_world = u / (np.linalg.norm(u) + 1e-9)
        self.range_m = range_m
        self.conf = conf
        self.t = t

    def __repr__(self):
        r = f"{self.range_m:.1f}m" if self.range_m else "no-range"
        return f"Bearing(u={np.round(self.u_world, 3)}, {r}, conf={self.conf:.2f})"


class VisionSource:
    """Base class for anything that produces a :class:`Bearing`."""

    def bearing(self, t) -> Bearing | None:
        raise NotImplementedError

    # Retained for compatibility with the original kit's signature.
    def get_bearing(self, t, attitude_q) -> Bearing | None:
        raise NotImplementedError

    @staticmethod
    def pixel_to_world_bearing(px, py, cam_intrinsics, R_body_to_world, cam_mount_R=None):
        fx, fy = cam_intrinsics["fx"], cam_intrinsics["fy"]
        cx, cy = cam_intrinsics["cx"], cam_intrinsics["cy"]
        ray_cam = np.array([(px - cx) / fx, (py - cy) / fy, 1.0])
        ray_cam /= np.linalg.norm(ray_cam)
        ray_body = cam_mount_R @ ray_cam if cam_mount_R is not None else ray_cam
        u_world = R_body_to_world @ ray_body
        return u_world / (np.linalg.norm(u_world) + 1e-9)
