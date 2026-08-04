"""Quaternion helpers. Hamilton convention, (w, x, y, z), scalar first.

Frames used throughout the stack:
  body  FRD — x forward (nose), y right, z down
  world NED — x north,   y east,  z down

``q`` always rotates a body vector into the world frame: ``v_world = q_to_R(q) @ v_body``.
"""

from __future__ import annotations

import math

import numpy as np


def q_normalize(q):
    q = np.asarray(q, float)
    return q / (np.linalg.norm(q) + 1e-12)


def q_mult(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def q_conj(q):
    w, x, y, z = q
    return np.array([w, -x, -y, -z])


def q_to_R(q):
    """Rotation matrix taking body-frame vectors to world-frame vectors."""
    w, x, y, z = q_normalize(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def euler_to_q(roll_deg: float, pitch_deg: float, yaw_deg: float):
    """Aerospace 3-2-1 (yaw, then pitch, then roll) euler angles to quaternion.

    Angles are in degrees: roll positive right-wing-down, pitch positive
    nose-up, yaw positive clockwise seen from above (compass heading).
    """
    r, p, y = (math.radians(a) for a in (roll_deg, pitch_deg, yaw_deg))
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def q_to_euler(q):
    """Inverse of :func:`euler_to_q`. Returns (roll, pitch, yaw) in degrees."""
    w, x, y, z = q_normalize(q)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    s = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(s)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return tuple(math.degrees(a) for a in (roll, pitch, yaw))


def unit(v):
    v = np.asarray(v, float)
    return v / (np.linalg.norm(v) + 1e-9)
