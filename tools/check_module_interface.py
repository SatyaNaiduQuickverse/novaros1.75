#!/usr/bin/env python3
"""Verify a command module against the interface contract, without hardware.

    python3 tools/check_module_interface.py companion.command_module
    python3 tools/check_module_interface.py guidance_v3 --path /opt/vendor
    python3 tools/check_module_interface.py guidance_v3 --class Guidance --ticks 500

Exercises the module against a mock FC, IMU and vision source and reports what
it actually did. This is behavioural, not introspective, on purpose: a compiled
extension carries no useful Python signature metadata, so reading signatures
proves very little about a `.so`. What matters is whether it survives a missing
bearing, whether it goes through set_stick(), and what it costs per tick.

Exit code 0 if every check passed, 1 otherwise. Nothing here touches a real
flight controller.

See docs/COMMAND_MODULE_INTERFACE.md for the contract being checked.
"""

from __future__ import annotations

import argparse
import importlib
import os
import struct
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from companion.config import Config                      # noqa: E402
from companion.fc_link import FCLink                     # noqa: E402
from companion.math_utils import euler_to_q              # noqa: E402
from companion.msp import (                              # noqa: E402
    MSP_MOTOR, MSP_RC, MSP_SET_RAW_RC, MSP_STATUS, MSPError,
)
from companion.vision_interface import Bearing           # noqa: E402

OK, BAD, WARN = "[ OK ]", "[FAIL]", "[WARN]"
ARM_BIT, OVERRIDE_BIT = 0, 26


class SpyMSP:
    """Answers state queries and records every transaction the module causes."""

    def __init__(self):
        self.calls = []
        self.rc_frames = []
        self.crc_errors = 0

    def request(self, cmd, payload=b"", timeout=None):
        self.calls.append(cmd)
        if cmd == MSP_SET_RAW_RC:
            self.rc_frames.append(struct.unpack(f"<{len(payload) // 2}H", payload))
            return b""
        if cmd == MSP_STATUS:
            flags = (1 << ARM_BIT) | (1 << OVERRIDE_BIT)
            return (struct.pack("<HHH", 500, 0, 0) + struct.pack("<I", flags)
                    + bytes([0]) + struct.pack("<H", 10) + struct.pack("<H", 125)
                    + bytes([0]) + bytes([0]) + struct.pack("<I", 0))
        if cmd == MSP_MOTOR:
            return struct.pack("<4H", *([1050] * 4))
        if cmd == MSP_RC:
            return struct.pack("<9H", *([1500] * 3 + [1000] + [1000] * 5))
        raise MSPError(f"unexpected cmd {cmd}")

    def close(self):
        pass


class SpyIMU:
    def __init__(self):
        self.calls = 0

    def get_state(self):
        self.calls += 1
        return (euler_to_q(2.0, -3.0, 45.0), np.zeros(3),
                np.array([0.0, 0.0, -9.80665]))

    def stale(self):
        return False


class SpyVision:
    """Returns a bearing most ticks and None the rest — losing lock is normal."""

    def __init__(self, drop_every=3):
        self.calls = 0
        self.drop_every = drop_every
        self.nones = 0

    def bearing(self, t):
        self.calls += 1
        if self.calls % self.drop_every == 0:
            self.nones += 1
            return None
        return Bearing([1.0, 0.05, -0.02], range_m=42.0, conf=0.83, t=t)


class SpyLogger:
    def __init__(self):
        self.records = 0

    def log(self, **fields):
        self.records += 1
        return fields

    def event(self, name, **fields):
        self.records += 1
        return fields


def load_class(module_name: str, class_name: str | None, path: str | None):
    if path:
        sys.path.insert(0, os.path.abspath(path))
    mod = importlib.import_module(module_name)
    if class_name:
        if not hasattr(mod, class_name):
            raise SystemExit(f"{BAD} {module_name} has no class {class_name!r}")
        return getattr(mod, class_name), mod
    if hasattr(mod, "CommandModule"):
        return mod.CommandModule, mod
    cands = [getattr(mod, n) for n in dir(mod)
             if isinstance(getattr(mod, n, None), type) and hasattr(getattr(mod, n), "step")]
    if len(cands) == 1:
        print(f"{WARN} no CommandModule; using discovered class {cands[0].__name__!r}")
        return cands[0], mod
    raise SystemExit(f"{BAD} could not find a command-module class; pass --class")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("module", help="importable module name (dotted or top-level)")
    p.add_argument("--class", dest="class_name", default=None)
    p.add_argument("--path", default=None, help="directory to add to sys.path")
    p.add_argument("--ticks", type=int, default=200)
    p.add_argument("--duration", type=float, default=30.0,
                   help="simulated engagement length to sweep t across, seconds; "
                        "must exceed the module's longest phase or late phases "
                        "never execute")
    p.add_argument("--rate", type=float, default=50.0, help="control rate for the budget check")
    a = p.parse_args()

    cls, mod = load_class(a.module, a.class_name, a.path)
    print(f"module   : {getattr(mod, '__file__', '?')}")
    print(f"class    : {cls.__name__}")
    print(f"compiled : {not str(getattr(mod, '__file__', '')).endswith('.py')}")
    print()

    fails, warns = [], []

    def check(cond, label, detail="", warn_only=False):
        mark = OK if cond else (WARN if warn_only else BAD)
        print(f"  {mark} {label}" + (f" — {detail}" if detail else ""))
        if not cond:
            (warns if warn_only else fails).append(label)
        return cond

    # --- construction -------------------------------------------------------
    print("construction")
    from companion.config import load as _load
    try:
        _cfg = _load()
    except Exception:
        _cfg = Config()
    ceiling = getattr(_cfg.fc, "msp_txn_per_sec", 99.0)
    fc = FCLink(Config())
    fc.msp = SpyMSP()
    fc.arm_bit, fc.override_bit = ARM_BIT, OVERRIDE_BIT
    imu, vision, logger = SpyIMU(), SpyVision(), SpyLogger()
    # Record any attempt to arm rather than letting PermissionError propagate,
    # so a module that tries is reported instead of just crashing the run.
    arm_attempts = []
    fc.arm = lambda on=True: arm_attempts.append(on)
    try:
        g = cls(fc, imu, vision, logger=logger)
        check(True, "CommandModule(fc, imu, vision, logger=...)")
    except Exception as e:
        check(False, "CommandModule(fc, imu, vision, logger=...)", repr(e))
        return 1
    try:
        cls(fc, imu, vision)
        check(True, "logger is optional")
    except Exception as e:
        check(False, "logger is optional", repr(e))

    # --- engagement lifecycle ----------------------------------------------
    print("\nengagement")
    has_engage = hasattr(g, "engage")
    check(has_engage, "exposes engage()",
          "phase clock starts here; t is seconds since this call"
          if has_engage else "no engage() — assuming the phase clock starts at "
                             "the first step()", warn_only=True)
    if has_engage:
        try:
            g.engage()
            check(True, "engage() completed")
        except Exception as e:
            check(False, "engage() completed", repr(e))
            return 1

    # --- per-tick behaviour -------------------------------------------------
    print("\nper-tick behaviour")
    calls0 = len(fc.msp.calls)
    worst, total, ret_dicts, errors = 0.0, 0.0, 0, []
    ran = 0
    # t is SWEPT across a realistic engagement, not taken from the wall clock.
    # A module with time-based phases (boost, terminal) only reaches its later
    # phases if t actually advances — calling step() in a tight loop with a
    # near-constant t would certify the first phase and nothing else.
    dt_sim = a.duration / max(a.ticks, 1)
    print(f"  sweeping t across 0..{a.duration:.0f}s in {a.ticks} steps "
          f"({dt_sim * 1000:.0f} ms of simulated engagement per tick)")
    for i in range(a.ticks):
        t = i * dt_sim
        t0 = time.perf_counter()
        try:
            r = g.step(t)
        except Exception as e:
            errors.append(f"tick {i} (t={t:.2f}s): {e!r}")
            break
        dt = time.perf_counter() - t0
        worst = max(worst, dt)
        total += dt
        ran += 1
        if isinstance(r, dict):
            ret_dicts += 1

    check(not errors, f"survived {a.ticks} ticks",
          f"stopped after {ran}: {errors[0]}" if errors else "")
    check(vision.nones > 0 and not errors,
          "survived vision.bearing() returning None",
          f"{vision.nones} of {vision.calls} ticks had no measurement")
    check(ran > 0 and ret_dicts == ran, "step() returns a dict",
          f"{ret_dicts}/{ran}", warn_only=True)

    budget_ms = 1000.0 / a.rate
    mean_ms = 1000.0 * total / max(ran, 1)
    check(worst * 1000 < budget_ms, "step() fits the control budget",
          f"mean {mean_ms:.2f} ms, worst {worst * 1000:.2f} ms vs {budget_ms:.1f} ms at "
          f"{a.rate:.0f} Hz")

    # --- how it talked to the FC -------------------------------------------
    print("\nFC usage")
    calls = fc.msp.calls[calls0:]
    per_tick = len(calls) / max(ran, 1)
    rc_frames = fc.msp.rc_frames
    # NB: this cannot tell set_stick() from a raw MSP_SET_RAW_RC write. The
    # channel-count and clamp checks below are what actually catch a bypass.
    check(len(rc_frames) > 0, "emitted RC frames", f"{len(rc_frames)} frames")
    check(all(len(f) == 4 for f in rc_frames), "only 4 channels transmitted",
          "aux channels must never be written")
    if rc_frames:
        thr = [f[2] for f in rc_frames]          # AETR: throttle is index 2
        sticks = [v for f in rc_frames for v in (f[0], f[1], f[3])]
        check(max(thr) <= fc.limits.thr_cap, "throttle within cap",
              f"max {max(thr)} vs cap {fc.limits.thr_cap}")
        check(all(abs(v - 1500) <= fc.limits.max_deflect for v in sticks),
              "deflection within limit",
              f"max {max(abs(v - 1500) for v in sticks)} vs {fc.limits.max_deflect}")

    extra = [c for c in calls if c != MSP_SET_RAW_RC]
    check(per_tick <= 1.2, "MSP transactions per tick",
          f"{per_tick:.2f}/tick -> {per_tick * a.rate:.0f} txn/s at {a.rate:.0f} Hz "
          f"(ceiling ~{ceiling:.0f} shared, per-board)", warn_only=per_tick <= 2.0)
    if extra:
        from collections import Counter
        print(f"        non-RC transactions in the tick path: "
              f"{dict(Counter(extra))} — prefer the cached read_* accessors")

    check(logger.records > 0, "used the logger", f"{logger.records} records",
          warn_only=True)
    check(imu.calls > 0, "read the IMU", f"{imu.calls} calls")

    # --- things it must not do ---------------------------------------------
    print("\nprohibited actions")
    check(not arm_attempts, "did not attempt to arm",
          f"called fc.arm() {len(arm_attempts)}x — ARM belongs to the pilot"
          if arm_attempts else "")

    print()
    if fails:
        print(f"{BAD} {len(fails)} check(s) failed: " + "; ".join(fails))
    if warns:
        print(f"{WARN} {len(warns)} advisory: " + "; ".join(warns))
    if not fails:
        print(f"{OK} interface contract satisfied")
    print("\nNOTE: this checks behaviour through the documented seam only. It "
          "cannot\nprove a compiled module does not bypass it at runtime — see "
          "the trust\nboundary section of docs/COMMAND_MODULE_INTERFACE.md.")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
