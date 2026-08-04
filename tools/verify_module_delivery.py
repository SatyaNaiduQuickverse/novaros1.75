#!/usr/bin/env python3
"""Acceptance check for a delivered compiled command module.

    python3 tools/verify_module_delivery.py <path/to/command_module.so> <sha256>

Two things, in order, and the order matters:

  1. The artifact is the one that was sent (SHA-256).
  2. The binary imports and behaves correctly on THIS machine.

**What this does not prove.** It feeds the module a known-good attitude
quaternion that this script constructs itself, so it verifies the module's
sign convention and its call pattern. It says nothing about whether our IMU
produces a correct quaternion — a wrong axis map or a flipped sign upstream
would sail through every check here and then drive the airframe into the tilt.
That is the separate tilt/sign calibration (`bringup imu32 --axis-map`), and
this tool deliberately refuses to imply otherwise.

Self-contained: numpy only, no FC, no hardware.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import os
import sys

import numpy as np

OK, BAD, WARN = "[ OK ]", "[FAIL]", "[WARN]"


def q_roll(deg):
    a = np.radians(deg) / 2.0
    return np.array([np.cos(a), np.sin(a), 0.0, 0.0])


def q_pitch(deg):
    a = np.radians(deg) / 2.0
    return np.array([np.cos(a), 0.0, np.sin(a), 0.0])


class SpyFC:
    """Records the last set_stick call and refuses to arm, like the real one."""

    def __init__(self):
        self.k = {}
        self.calls = 0
        self.arm_attempts = 0

    def set_stick(self, roll=None, pitch=None, yaw=None, throttle=None):
        self.k = {k: v for k, v in
                  (("roll", roll), ("pitch", pitch), ("yaw", yaw),
                   ("throttle", throttle)) if v is not None}
        self.calls += 1

    def arm(self, on=True):
        self.arm_attempts += 1


class SpyIMU:
    def __init__(self):
        self.q = q_roll(0.0)

    def get_state(self):
        return self.q, np.zeros(3), np.array([0.0, 0.0, -9.81])


def sha256_of(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def load_module(path):
    name = os.path.basename(path).split(".")[0]
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("so_path")
    ap.add_argument("sha256", nargs="?", default=None,
                    help="expected digest; omitted only for a dry run")
    a = ap.parse_args()

    fails = []

    def check(cond, label, detail=""):
        print(f"  {OK if cond else BAD} {label}" + (f" — {detail}" if detail else ""))
        if not cond:
            fails.append(label)
        return cond

    print(f"file: {a.so_path}")
    if not os.path.exists(a.so_path):
        print(BAD, "not found")
        return 1
    print(f"size: {os.path.getsize(a.so_path)} bytes")

    print("\nprovenance")
    got = sha256_of(a.so_path)
    print(f"  sha256: {got}")
    if a.sha256:
        check(got.lower() == a.sha256.lower().strip(), "SHA-256 matches the sent value",
              "" if got.lower() == a.sha256.lower().strip() else f"expected {a.sha256}")
        if fails:
            print("\nRefusing to go further — this is not the binary that was sent.")
            return 1
    else:
        print(f"  {WARN} no expected digest given — provenance UNVERIFIED")

    print("\nimport")
    try:
        mod = load_module(a.so_path)
        check(True, "imports on this machine",
              f"python {sys.version.split()[0]} {os.uname().machine}")
    except Exception as e:
        check(False, "imports on this machine", repr(e))
        return 1
    public = [n for n in dir(mod) if not n.startswith("_")]
    print(f"  public symbols: {public}")
    check(hasattr(mod, "CommandModule"), "exposes CommandModule")

    print("\nconstruction")
    fc, imu = SpyFC(), SpyIMU()
    try:
        g = mod.CommandModule(fc, imu, None, None)
        check(True, "CommandModule(fc, imu, None, None)")
    except Exception as e:
        check(False, "CommandModule(fc, imu, None, None)", repr(e))
        return 1

    print("\nbehaviour (attitude supplied by this script, not by our IMU)")
    imu.q = q_roll(0.0)
    rc = g.step(0.0)
    check(isinstance(rc, dict) and "roll" in rc, "step() returns a dict with 'roll'",
          str(rc))
    level_roll = rc.get("roll") if isinstance(rc, dict) else None

    imu.q = q_roll(+15.0)
    rc_r = g.step(0.02)
    check(rc_r["roll"] < 1500, "roll +15 deg -> roll correction < 1500",
          f"got {rc_r['roll']}")

    imu.q = q_roll(-15.0)
    rc_l = g.step(0.04)
    check(rc_l["roll"] > 1500, "roll -15 deg -> roll correction > 1500 (opposite)",
          f"got {rc_l['roll']}")

    imu.q = q_pitch(+15.0)
    rc_p = g.step(0.06)
    check(rc_p["pitch"] != level_roll, "pitch tilt moves the pitch channel",
          f"got {rc_p['pitch']}")

    print("\ncall pattern")
    check("throttle" not in fc.k, "no throttle emitted (pilot keeps ch3)",
          f"last set_stick kwargs: {sorted(fc.k)}")
    check(fc.arm_attempts == 0, "never calls arm()")
    check(fc.calls > 0, "commands via set_stick()", f"{fc.calls} calls")

    print()
    if fails:
        print(f"{BAD} {len(fails)} check(s) failed: " + "; ".join(fails))
        return 1
    print(f"{OK} delivered binary is correct on this machine.")
    print()
    print("THIS DOES NOT CLEAR THE AIRFRAME. The attitude above was synthesised")
    print("here. Whether our IMU hands the module a correctly-framed quaternion")
    print("is the tilt/sign calibration — a wrong axis or sign passes every")
    print("check above and then drives INTO the tilt. Run:")
    print("    python3 -m tools.bringup imu32 --axis-map")
    print("and confirm imu32.verified is true before this drives a motor.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
