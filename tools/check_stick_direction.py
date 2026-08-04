#!/usr/bin/env python3
"""Which pitch/roll channel value means NOSE DOWN? Ask the pilot's transmitter.

    python3 tools/check_stick_direction.py

The self-level module emits channel VALUES. Knowing our attitude is correct
does not tell us what 1400 on the pitch channel does to this airframe — that
depends on the transmitter, the channel map, and any reversals, none of which
are visible from the companion side.

The pilot's own sticks settle it. Forward stick is nose down (a multirotor
tips its thrust vector toward the ground to accelerate forward), so whatever
the pitch channel reads with the stick forward IS the nose-down direction.

Reads MSP_RC only. Streams nothing, arms nothing, spins nothing. The override
switch must be DOWN so the sticks belong to the pilot — with it up the
companion owns those channels and this would measure its own idle frame.
"""

from __future__ import annotations

import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from companion.config import load                    # noqa: E402
from companion.fc_link import FCLink                  # noqa: E402

OK, BAD, WARN = "  [ OK ]", "  [FAIL]", "  [WARN]"
# MSP_RC returns roll, pitch, YAW, THROTTLE — not the AETR wire order. Getting
# this backwards printed "thr 1498 / yaw 989" for a centred yaw and a low
# throttle. See CLAUDE.md; the two orders are genuinely different.
RC_ROLL, RC_PITCH, RC_YAW, RC_THROTTLE = 0, 1, 2, 3

MOVED_US = 150          # how far from centre counts as "held"
SETTLE_S = 1.0          # how long it must stay there before recording
RECORD_S = 2.0
WAIT_S = 25.0

PHASES = [
    ("PITCH stick fully FORWARD (away from you)",
     "the input that would fly it forward", RC_PITCH),
    ("PITCH stick fully BACK (toward you)",
     "the input that would fly it backward", RC_PITCH),
    ("ROLL stick fully RIGHT",
     "the input that would fly it right", RC_ROLL),
]


def hold(fc, label, hint, idx):
    """Wait for the operator, do not race a clock.

    The timed version failed because it demanded the stick be in position at a
    moment chosen by the script; the operator ran a phase behind throughout and
    the run measured a centred stick, then a stick mid-travel, then the wrong
    axis entirely. This waits for the channel to actually move and settle, so
    there is nothing to be late for.
    """
    print(f"\n>>> {label}")
    print(f"    ({hint})  — hold it there; recording starts by itself")
    deadline = time.time() + WAIT_S
    settled_since = None
    while time.time() < deadline:
        rc = fc.rc()[:4]
        if abs(rc[idx] - 1500) > MOVED_US:
            settled_since = settled_since or time.time()
            if time.time() - settled_since >= SETTLE_S:
                break
        else:
            settled_since = None
        time.sleep(0.05)
    else:
        raise RuntimeError(f"that stick never moved more than {MOVED_US} us from "
                           "centre — is the transmitter bound?")
    print("    got it — recording", flush=True)
    vals, end = [], time.time() + RECORD_S
    while time.time() < end:
        try:
            vals.append(fc.rc()[:4])
        except Exception:
            pass
        time.sleep(0.05)
    a = np.array(vals, float)
    m, sd = a.mean(axis=0), a.std(axis=0)
    print("    roll %4.0f   pitch %4.0f   yaw %4.0f   throttle %4.0f  (jitter %.0f us)"
          % (m[RC_ROLL], m[RC_PITCH], m[RC_YAW], m[RC_THROTTLE], sd.max()))
    print("    now CENTRE the sticks", flush=True)
    # Wait for centre so the next phase cannot trigger on this one's deflection.
    t = time.time() + 10
    while time.time() < t:
        if max(abs(v - 1500) for v in fc.rc()[:2]) < 60:
            break
        time.sleep(0.05)
    return m


def main():
    cfg = load()
    fc = FCLink(cfg).connect()
    try:
        ch = cfg.channels
        rc = fc.rc()
        print("rc now:", rc)
        ovr = rc[ch.override_index] if len(rc) > ch.override_index else None
        print(f"override switch ch{ch.override_index + 1} = {ovr} us")
        if ovr is not None and ovr >= ch.override_active_us:
            print(BAD, "override is UP — the companion owns ch1-4, so this would")
            print("       measure an idle frame rather than your sticks. Switch it")
            print("       DOWN and re-run.")
            return 1
        if len(rc) > 3 and max(abs(v - 1500) for v in rc[:2]) < 5 and rc[RC_THROTTLE] < 1000:
            print(WARN, "sticks look centred and throttle low — normal, but make")
            print("       sure the transmitter is actually bound before starting.")
        print("\nPROPS OFF. Nothing is streamed and nothing can arm; this reads only.")

        got = [hold(fc, label, hint, idx) for label, hint, idx in PHASES]
    finally:
        fc.close()

    fwd, back, right = got
    print("\n" + "=" * 64)
    dp = fwd[RC_PITCH] - back[RC_PITCH]
    if abs(dp) < 100:
        print(BAD, f"the pitch channel moved only {abs(dp):.0f} us between full")
        print("       forward and full back. Is the transmitter bound and were the")
        print("       sticks actually held? Nothing can be concluded.")
        return 1

    nose_down_high = fwd[RC_PITCH] > back[RC_PITCH]
    print(f"pitch: stick forward -> {fwd[RC_PITCH]:.0f}"
          f"    stick back -> {back[RC_PITCH]:.0f}")
    print(f"roll : stick right   -> {right[RC_ROLL]:.0f}    (centre is ~1500)")
    print()
    print("Forward stick is nose DOWN — a multirotor tips its thrust vector toward")
    print("the ground to accelerate forward. Therefore on THIS airframe:")
    print()
    print(f"    NOSE DOWN  =  pitch channel {'ABOVE' if nose_down_high else 'BELOW'} 1500")
    print(f"    NOSE UP    =  pitch channel {'BELOW' if nose_down_high else 'ABOVE'} 1500")
    print(f"    ROLL RIGHT =  roll  channel "
          f"{'ABOVE' if right[RC_ROLL] > 1500 else 'BELOW'} 1500")
    print()
    print("So a self-level module correcting a NOSE-UP tilt must command pitch")
    print(f"    {'ABOVE' if nose_down_high else 'BELOW'} 1500, and correcting "
          f"RIGHT-side-down must command roll")
    print(f"    {'BELOW' if right[RC_ROLL] > 1500 else 'ABOVE'} 1500.")
    print()
    print("Check that against the 3b panel in the calibration UI. If they")
    print("disagree, the module's sign is wrong for this airframe and it would")
    print("drive INTO the tilt — do not fly it.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
