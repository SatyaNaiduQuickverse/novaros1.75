import numpy as np


class Bearing:
    __slots__ = ('u_world', 'range_m', 'conf', 't')

    def __init__(self, u_world, range_m=None, conf=1.0, t=0.0):
        u = np.asarray(u_world, float)
        self.u_world = u / (np.linalg.norm(u) + 1e-09)
        self.range_m = range_m
        self.conf = conf
        self.t = t


class VisionSource:

    def get_bearing(self, t, attitude_q):
        raise NotImplementedError

    @staticmethod
    def pixel_to_world_bearing(px, py, cam_intrinsics, R_body_to_world, cam_mount_R=None):
        fx = cam_intrinsics['fx']; fy = cam_intrinsics['fy']
        cx = cam_intrinsics['cx']; cy = cam_intrinsics['cy']
        ray_cam = np.array([(px - cx) / fx, (py - cy) / fy, 1.0])
        ray_cam /= np.linalg.norm(ray_cam)
        ray_body = cam_mount_R @ ray_cam if cam_mount_R is not None else ray_cam
        u_world = R_body_to_world @ ray_body
        return u_world / (np.linalg.norm(u_world) + 1e-09)
