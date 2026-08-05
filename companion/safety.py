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

By default only channels 1-4 are transmitted, and the FC's
``msp_override_channels_mask = 15`` keeps every aux channel — including ARM —
with the pilot's transmitter regardless of what the companion sends.

⚠️ **Arming from the companion changes that**, and is off unless explicitly
enabled. See :func:`aetr_frame`'s ``arm`` argument and the notes on it: on this
board the override has NO timeout, so whatever is streamed on the ARM channel
is what the FC keeps applying. Disarm therefore has to be the RESTING state —
the value sent when nothing has asked for anything — rather than something the
code remembers to send.
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


# ARM channel values. LOW is what everything defaults to, everywhere.
ARM_LOW_US = 1000
ARM_HIGH_US = 2000
# Zero-based index of ARM within the outgoing frame. ch9 -> index 8. Channels
# 5..8 are padded with a neutral value; the FC ignores them unless their mask
# bits are set, and they are deliberately NOT configurable here — this function
# exists to make the frame layout un-gettable-wrong.
IDX_ARM = 8
AUX_NEUTRAL_US = 1000


def aetr_frame(roll: float, pitch: float, yaw: float, throttle: float,
               limits: Limits, arm: bool | None = None) -> bytes:
    """Pack one MSP_SET_RAW_RC payload in raw receiver (AETR) order.

    Arguments are in the intuitive roll/pitch/yaw/throttle order; the
    reordering and the clamping both happen here so a caller cannot get either
    wrong. A garbage input becomes centred sticks at the throttle floor.

    ``arm`` controls the frame LENGTH as well as its contents, and that is the
    safety-relevant part:

    * ``arm=None`` (default) packs **four channels**. The ARM channel is not
      present in the frame at all, so it cannot be set by accident, and the FC
      keeps taking ARM from the pilot's receiver. This is the normal case.
    * ``arm=False`` packs nine channels with ARM explicitly LOW.
    * ``arm=True`` packs nine channels with ARM HIGH.

    Note what ``arm=False`` is FOR. On this board the MSP override has no
    timeout, so the FC keeps applying the last frame it received forever. If
    the companion is streaming the ARM channel at all, then every frame that
    is not deliberately arming must actively say "disarmed" — a frame that
    merely omits it would leave the previous HIGH in force. Disarm is the
    resting state, sent continuously, not an event.
    """
    lo, hi = CENTER_US - limits.max_deflect, CENTER_US + limits.max_deflect
    core = (
        clamp(roll, lo, hi, CENTER_US),
        clamp(pitch, lo, hi, CENTER_US),
        clamp(throttle, limits.thr_floor, limits.thr_cap, limits.thr_floor),
        clamp(yaw, lo, hi, CENTER_US),
    )
    if arm is None:
        return struct.pack("<4H", *core)
    # Anything that is not exactly True disarms. A None, a NaN, a string, a
    # half-initialised object — all of them mean "not armed", because the only
    # safe way to read an unclear intention on this channel is as a refusal.
    armed = arm is True
    aux = [AUX_NEUTRAL_US] * (IDX_ARM - 4) + [ARM_HIGH_US if armed else ARM_LOW_US]
    return struct.pack("<%dH" % (4 + len(aux)), *core, *aux)


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
