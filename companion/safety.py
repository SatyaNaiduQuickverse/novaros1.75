"""Channel ordering and the command clamps, in one place.

Two things live here because getting either wrong is expensive:

1. **AETR ordering.** ``MSP_SET_RAW_RC`` (cmd 200) takes channels in RAW
   RECEIVER order, which with the standard AETR map puts THROTTLE at index 2:

       index   0      1       2        3
       meaning ROLL   PITCH   THROTTLE YAW

   ``MSP_RC`` (cmd 105) *returns* roll, pitch, yaw, throttle — throttle last.
   Mixing these up once streamed 1550 onto the throttle channel on this
   airframe. Build every outgoing frame with :func:`aetr_frame`; nothing else
   in the codebase packs channel bytes.

2. **The clamps.** Every command passes through :func:`aetr_frame`, so the
   throttle cap and deflection limit cannot be bypassed by a caller mistake.

Only channels 1-4 are ever transmitted. The FC is configured with
``msp_override_channels_mask = 15``, so aux channels — including ARM — stay
with the pilot's transmitter regardless of what the companion sends.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass

# --- outgoing frame indices (raw receiver order) ---
IDX_ROLL, IDX_PITCH, IDX_THROTTLE, IDX_YAW = 0, 1, 2, 3

# --- MSP_RC readback indices (roll, pitch, yaw, throttle, then aux) ---
RC_ROLL, RC_PITCH, RC_YAW, RC_THROTTLE = 0, 1, 2, 3

CENTER_US = 1500
THROTTLE_MIN_US = 1000


@dataclass(frozen=True)
class Limits:
    """Command envelope. ``bench`` is the props-off default.

    thr_cap 1100 is roughly 10% stick and is enough to see motors respond
    without generating meaningful thrust. Raise it only for tethered or
    free-flight integration, never for bench work.
    """

    thr_floor: int = 1000
    thr_cap: int = 1100
    max_deflect: int = 100      # us either side of centre on roll/pitch/yaw
    motor_abort: int = 1700     # any motor readback above this stops the stream
    profile: str = "bench"

    @staticmethod
    def bench() -> "Limits":
        return Limits()

    @staticmethod
    def named(profile: str, **overrides) -> "Limits":
        base = {
            "bench": dict(thr_floor=1000, thr_cap=1100, max_deflect=100, motor_abort=1700),
            "tethered": dict(thr_floor=1000, thr_cap=1300, max_deflect=150, motor_abort=1850),
            "flight": dict(thr_floor=1000, thr_cap=1800, max_deflect=400, motor_abort=2000),
        }
        if profile not in base:
            raise ValueError(f"unknown limits profile {profile!r}; have {sorted(base)}")
        return Limits(profile=profile, **{**base[profile], **overrides})


def clamp(v: float, lo: int, hi: int, fallback: int) -> int:
    """Clamp to [lo, hi]; a non-finite value degrades to ``fallback``.

    NaN and inf are realistic outputs of a guidance solve that divided by a
    zero range or lost its state. They must become a safe command, not an
    exception inside the streaming thread.
    """
    try:
        v = float(v)
    except (TypeError, ValueError):
        return fallback
    if not math.isfinite(v):
        return fallback
    return max(lo, min(hi, int(round(v))))


def aetr_frame(roll: float, pitch: float, yaw: float, throttle: float,
               limits: Limits) -> bytes:
    """Pack one 4-channel MSP_SET_RAW_RC payload in raw receiver (AETR) order.

    Arguments are in the intuitive roll/pitch/yaw/throttle order; the
    reordering and the clamping both happen here so a caller cannot get either
    wrong. A garbage input becomes centred sticks at the throttle floor.
    """
    lo, hi = CENTER_US - limits.max_deflect, CENTER_US + limits.max_deflect
    return struct.pack(
        "<4H",
        clamp(roll, lo, hi, CENTER_US),
        clamp(pitch, lo, hi, CENTER_US),
        clamp(throttle, limits.thr_floor, limits.thr_cap, limits.thr_floor),
        clamp(yaw, lo, hi, CENTER_US),
    )


IDLE_STICKS = {"roll": CENTER_US, "pitch": CENTER_US, "yaw": CENTER_US,
               "throttle": THROTTLE_MIN_US}


class AbortReason(str):
    """Why the RC stream stopped. Stopping hands control back to the pilot."""


ABORT_PILOT_DISARMED = AbortReason("pilot disarmed")
ABORT_OVERRIDE_RELEASED = AbortReason("pilot took control back")
ABORT_MOTOR_CAP = AbortReason("motor readback above cap")
ABORT_LINK_ERROR = AbortReason("serial/MSP error")
ABORT_RX_LOSS = AbortReason("pilot RC link lost")
ABORT_REQUESTED = AbortReason("stop requested")
ABORT_ENVELOPE = AbortReason("FC received a command outside the envelope")
