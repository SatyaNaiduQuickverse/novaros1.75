#!/usr/bin/env python3
"""Is the FC's attitude a trustworthy reference frame? Ask the FC itself.

    python3 tools/check_fc_attitude.py

The axis-map calibration treats the flight controller as ground truth for
"which way is the airframe pointing". That is only worth doing if the euler
convention we decode it with is actually right — and a wrong convention looks
exactly like a badly mounted ESP32: pairs that no single rotation explains.

So this compares the FC against ITSELF and leaves the ESP32 out entirely:

    predicted UP  =  -gravity direction derived from MSP_ATTITUDE roll/pitch
    measured  UP  =  the direction MSP_RAW_IMU's accelerometer actually reports

Both come from the same board. At rest they must agree, in every orientation.
If they agree, the convention is right and any disagreement with the ESP32 is
the ESP32's (mounting, rigidity, its own calibration). If they diverge only
once tilted, the convention is wrong and no amount of re-tumbling will help.

Reads only. Never streams RC, never arms.
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from companion.config import load                            # noqa: E402
from companion.fc_link import FCLink                          # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bringup import _gravity_in_body_from_fc                  # noqa: E402

OK, BAD, WARN = "  [ OK ]", "  [FAIL]", "  [WARN]"

# (label, hint, axis, sign) — the axis/sign is what GRAVITY must do in body
# FRD for that pose, and the pose name is the ground truth: the operator knows
# they are holding it nose up. Checking the FC against that is what validates
# the euler convention itself, which comparing the FC to its own accelerometer
# cannot do (a sign error in either looks identical).
#
# FRD is x forward, y right, z down. Nose up tilts world-down toward the tail,
# so gravity gains -x. Right side down tilts it toward the right wing: +y.
POSES = [
    ("LEVEL",                    "sitting flat, as it flies",  2, +1),
    ("NOSE UP about 30-45 deg",  "nose toward the sky",        0, -1),
    ("ROLL RIGHT about 30-45 deg", "right side down",          1, +1),
    ("NOSE DOWN about 30-45 deg", "nose toward the ground",    0, +1),
]


def sample(fc, n=25):
    """Mean attitude and mean accel direction over n reads, both from the FC."""
    atts, accs = [], []
    for _ in range(n):
        att = fc.attitude()
        acc, _ = fc.raw_imu()
        atts.append([att["roll"], att["pitch"]])
        accs.append(np.asarray(acc, float))
    att = np.mean(atts, axis=0)
    acc = np.mean(accs, axis=0)
    spread = float(np.std(atts, axis=0).max())
    return {"roll": float(att[0]), "pitch": float(att[1])}, acc, spread


def main():
    cfg = load()
    fc = FCLink(cfg).connect()
    rows, worst = [], 0.0
    try:
        print("PROPS OFF. Four poses. Precision does not matter — 30 deg is plenty.\n")
        for name, hint, axis, want in POSES:
            try:
                input(f">>> Hold the airframe {name} ({hint}), still.\n"
                      f"    press ENTER when steady ")
            except (EOFError, KeyboardInterrupt):
                print("\naborted")
                return 1
            att, acc, spread = sample(fc)
            n = float(np.linalg.norm(acc))
            if n < 1e-6:
                print(BAD, "no accelerometer reading from the FC")
                return 1
            meas_up = acc / n
            pred_up = -_gravity_in_body_from_fc(att)
            ang = math.degrees(math.acos(np.clip(np.dot(meas_up, pred_up), -1, 1)))
            worst = max(worst, ang)
            # Does the FC's own attitude put gravity where this POSE says it
            # must be? Ground truth is the operator's hands, not another sensor.
            pred_down = -pred_up
            got = float(pred_down[axis])
            conv_ok = (abs(got) > 0.25 and (got > 0) == (want > 0)) if axis != 2 \
                else got > 0.8
            rows.append((name, att, meas_up, pred_up, ang, spread, conv_ok, axis,
                         want, got))
            print(f"    FC attitude   roll {att['roll']:+7.1f}  pitch {att['pitch']:+7.1f}"
                  f"   (moved {spread:.1f} deg during the read)")
            print(f"    measured  UP  {np.round(meas_up, 3)}   |a| {n:.2f} m/s^2")
            print(f"    predicted UP  {np.round(pred_up, 3)}")
            print(f"    disagreement  {ang:.1f} deg")
            axname = "xyz"[axis]
            print(f"    convention    gravity {axname} = {got:+.3f}, "
                  f"pose requires {'+' if want > 0 else '-'}ve"
                  f"   {'OK' if conv_ok else '<-- WRONG SIGN'}\n")
    finally:
        fc.close()

    print("=" * 66)
    conv_bad = [r[0] for r in rows if not r[6]]
    for name, att, _, _, ang, _, conv_ok, axis, want, got in rows:
        mark = OK if (ang < 10 and conv_ok) else BAD
        print(f"{mark} {name:<28} disagree {ang:5.1f} deg   "
              f"gravity {'xyz'[axis]} {got:+.2f} "
              f"({'ok' if conv_ok else 'WRONG SIGN'})")
    print("=" * 66)
    if conv_bad:
        print(BAD, "the FC does not put gravity where the pose says it must:")
        for n in conv_bad:
            print(f"       - {n}")
        print("       The euler decode in _gravity_in_body_from_fc has a sign or")
        print("       axis wrong for THIS firmware. Fix it before re-tumbling —")
        print("       the axis map would inherit the error, and more calibration")
        print("       runs cannot detect it.")
        return 1
    if worst < 10:
        print(OK, f"the FC agrees with its own accelerometer everywhere "
                  f"(worst {worst:.1f} deg).")
        print("       The euler convention is right, so it is sound ground truth")
        print("       for the axis map. A large residual there is then the")
        print("       ESP32's: check it cannot shift relative to the airframe.")
        return 0
    level = rows[0][4] if rows else 0.0
    print(BAD, f"the FC disagrees with its own accelerometer by up to {worst:.1f} deg.")
    if level < 10:
        print("       It agrees LEVEL but not tilted, which is the signature of a")
        print("       sign or axis error in the euler decode — not a hardware")
        print("       fault. Fix _gravity_in_body_from_fc before re-tumbling;")
        print("       more calibration runs cannot help.")
    else:
        print("       It disagrees even LEVEL, so suspect the FC's own")
        print("       accelerometer calibration (Betaflight Configurator) or")
        print("       cfg.imu.board_to_frd, which is still unverified.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
