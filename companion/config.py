"""Vehicle configuration: one file, loaded once, no constants buried in modules.

Defaults below match the airframe verified on this Pi (SpeedyBee F7 V3,
Betaflight 26.6.1 cloud build with MSP Override). ``config/vehicle.yaml``
overrides any of them; nothing here needs the file to exist.

    from companion.config import load
    cfg = load()                        # config/vehicle.yaml if present
    cfg = load("config/bench.yaml")
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field, asdict

from .safety import Limits

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_CONFIG = os.path.join(_REPO, "config", "vehicle.yaml")


@dataclass
class FCConfig:
    # "auto" resolves the single Betaflight board under /dev/serial/by-id/.
    # Pin the full by-id path once more than one board can be plugged in:
    # the bench board is 3678336D3333, the drone board is 366D335B3333.
    port: str = "auto"
    baud: int = 115200          # ignored on USB VCP, required for real UARTs
    timeout_s: float = 1.0
    # Measured per board — Betaflight services MSP from a scheduled task, so
    # the round-trip cost is set by the firmware/target, not by USB. Measure
    # with tools/find_fc.py --bench. F722/BTFL26.6.1: ~99. F405/BTFL4.5.1: ~62.
    msp_txn_per_sec: float = 62.0
    rc_hz: float = 50.0         # override frame rate; BF times out around 250 ms
    telemetry_hz: float = 12.0
    watchdog_hz: float = 3.0    # safety readbacks while streaming
    # Read MSP_RC back during each watchdog check and confirm the FC actually
    # received values inside the envelope. Catches anything sharing this
    # process that bypasses set_stick() — see FCLink._verify_envelope.
    verify_wire: bool = True


@dataclass
class ChannelConfig:
    """Switch map for this airframe and this pilot transmitter.

    Indices are zero-based into MSP_RC. The companion never transmits any of
    these; they are read-only, used to know who currently has control.
    """

    override_index: int = 7     # ch8  / AUX4 — MSP OVERRIDE switch
    arm_index: int = 8          # ch9  / AUX5 — ARM. Never streamed, never spoofed.
    angle_index: int = 4        # ch5  — ANGLE mode, active when LOW (900-1200)
    override_active_us: int = 1700
    # Must match msp_override_channels_mask on the FC. Bit N = TX channel N+1:
    #   bit0 ch1 roll, bit1 ch2 pitch, bit2 ch3 throttle, bit3 ch4 yaw
    #   15 (0b1111) companion owns roll/pitch/yaw/throttle
    #   11 (0b1011) companion owns roll/pitch/yaw; PILOT keeps throttle
    # Getting this wrong is not cosmetic: the wire-envelope check would judge
    # the pilot's own stick against the companion's limits and false-abort.
    override_mask: int = 15


@dataclass
class IMUCal:
    """MSP_RAW_IMU decoding. Verify with ``bringup calib-imu`` before trusting it.

    ``acc_per_g``: raw units per 1 g. Betaflight reports ``accADC / 4`` when
    ``acc_1G > 512``, so a 16 g gyro (acc_1G 2048) gives 512. The SITL build
    gives 256. Measure it rather than assuming.

    ``gyro_units``: modern Betaflight sends degrees/second directly ("dps").
    "lsb2000" applies the 1/16.4 MultiWii-era scale instead.

    ``board_to_frd``: signed axis permutation from the FC's sensor frame to
    body FRD (x forward, y right, z down). Entries are (source index, sign).
    The default assumes the FC reports x-forward / y-left / z-up.
    UNVERIFIED on this airframe — ``bringup calib-imu`` prints the correct map.
    """

    acc_per_g: float = 512.0
    gyro_units: str = "dps"
    board_to_frd: tuple = ((0, 1.0), (1, -1.0), (2, -1.0))
    verified: bool = False
    # Attitude sign conventions, same caveat.
    roll_sign: float = 1.0
    pitch_sign: float = 1.0
    yaw_sign: float = 1.0


@dataclass
class CameraCal:
    """Intrinsics and the camera-in-body mounting basis (boresight).

    Replace fx/fy/cx/cy with a real OpenCV checkerboard calibration; the
    defaults are a plausible guess for a 1280x720 sensor and will silently bias
    every bearing until measured.
    """

    width: int = 1280
    height: int = 720
    fx: float = 1650.0
    fy: float = 1650.0
    cx: float = 640.0
    cy: float = 360.0
    dist_coeffs: tuple = ()             # (k1,k2,p1,p2,k3) — empty = no undistort
    # camera axes expressed in body FRD. Default: boresight forward,
    # image-right = body right, image-up = body up (= -z in FRD).
    cam_fwd: tuple = (1.0, 0.0, 0.0)
    cam_right: tuple = (0.0, 1.0, 0.0)
    cam_up: tuple = (0.0, 0.0, -1.0)
    object_size_m: float = 0.4          # widest dimension, for the size-range hint
    min_box_px: int = 26                # below this a range estimate is noise
    verified: bool = False


@dataclass
class ESP32IMUCal:
    """Dedicated companion IMU: MPU6500 on an ESP32-C6 (see companion/imu_esp32.py).

    This is the flight-article attitude source. ``FCIMU`` costs an MSP
    transaction per sample from a budget that already caps the control rate;
    this path costs the FC nothing.

    ``gyro_bias`` is in RAW COUNTS and is subtracted before scaling. It is large
    on this part (~9 dps on one axis) and must be measured — an uncorrected
    bias fights the gravity correction and leaves a standing attitude error.

    ``accel_per_g`` is counts per 1 g, measurable at rest in any orientation —
    but only if the die is well behaved, and this one is not: it read 2400
    counts for gravity on sensor z and 2001 on sensor x, both stationary. Use
    ``accel_offset`` + ``accel_per_g_axis`` from ``bringup imu32 --accel-cal``
    instead; ``accel_per_g`` is then only a fallback for the axes they do not
    cover. The offsets are what matter for attitude — the Mahony filter
    normalises the accel vector, so a common scale error cancels out entirely
    while an offset tilts the measured gravity direction.

    ``axis_map`` is (source index, sign) per FRD axis, mapping the SENSOR frame
    to airframe FRD. Depends entirely on how the board is mounted and needs a
    level surface plus tilts to determine — hence ``verified``.
    """

    enabled: bool = False
    port: str = ""                   # blank = auto-detect the Espressif device
    baud: int = 921600
    gyro_bias: tuple = (0.0, 0.0, 0.0)
    accel_per_g: float = 2048.0
    accel_offset: tuple = (0.0, 0.0, 0.0)   # raw counts, sensor frame
    accel_per_g_axis: tuple = ()            # empty -> fall back to accel_per_g
    axis_map: tuple = ((0, 1.0), (1, 1.0), (2, 1.0))
    kp: float = 2.0                  # gravity correction gain
    ki: float = 0.05                 # residual bias integrator
    verified: bool = False


@dataclass
class ModuleConfig:
    """The compiled command module, pinned.

    ``sha256`` is the digest of the binary that was accepted after the delivery
    check. It is verified at load, so a silently swapped or re-delivered .so is
    caught rather than flown.

    Note on what this proves: the supplier builds on x86 and could not give an
    independently-computed aarch64 digest, so this value originates from the
    delivered file itself and does NOT constitute external certification. What
    established identity was the behavioural fingerprint below, reproduced here
    before the supplier quoted it. Treat the digest as change-detection, not
    provenance.
    """

    so_path: str = "companion/command_module.cpython-313-aarch64-linux-gnu.so"
    sha256: str = ""
    verify_on_load: bool = True


@dataclass
class GuidanceConfig:
    """Out-of-process guidance host. See companion/guidance_proxy.py.

    ``cert_sha256`` is mandatory to launch: the connector hashes the binary
    itself and refuses to start if it does not match, so an unpinned or swapped
    .so never runs.
    """

    enabled: bool = False
    host_module: str = "guidance_host"   # imported in the CHILD only
    so_dir: str = "/opt/nova/guidance"
    so_path: str = ""                    # set when so_dir holds several .so files
    cert_sha256: str = ""                # from the build; no default on purpose
    start_timeout_s: float = 10.0
    poll_timeout_s: float = 0.005        # 5 ms; ~47x margin inside a 20 ms tick
    init_opts: dict = field(default_factory=dict)


@dataclass
class Config:
    fc: FCConfig = field(default_factory=FCConfig)
    channels: ChannelConfig = field(default_factory=ChannelConfig)
    imu: IMUCal = field(default_factory=IMUCal)
    camera: CameraCal = field(default_factory=CameraCal)
    imu32: ESP32IMUCal = field(default_factory=ESP32IMUCal)
    module: ModuleConfig = field(default_factory=ModuleConfig)
    guidance: GuidanceConfig = field(default_factory=GuidanceConfig)
    limits: Limits = field(default_factory=Limits.bench)
    log_dir: str = "~/logs"
    source: str = "<defaults>"

    def as_dict(self) -> dict:
        return asdict(self)

    def unverified(self) -> list[str]:
        """Calibrations still on their factory guess. Bring-up must clear these."""
        pending = []
        if not self.imu.verified:
            pending.append("imu (acc scale / axis map) — run: bringup calib-imu")
        if not self.camera.verified:
            pending.append("camera (intrinsics / boresight) — run: bringup vision")
        if self.imu32.enabled and not self.imu32.verified:
            pending.append("esp32 imu (axis map) — run: bringup imu32 --calibrate")
        if self.imu32.enabled and len(self.imu32.accel_per_g_axis) != 3:
            pending.append("esp32 accel (per-axis offset/scale, this part needs "
                           "it) — run: bringup imu32 --accel-cal")
        return pending


def _apply(obj, data: dict, path: str):
    for key, val in data.items():
        if not hasattr(obj, key):
            raise ValueError(f"{path}: unknown setting {key!r}")
        cur = getattr(obj, key)
        if hasattr(cur, "__dataclass_fields__") and isinstance(val, dict):
            _apply(cur, val, f"{path}.{key}")
        elif isinstance(cur, tuple) and isinstance(val, list):
            setattr(obj, key, tuple(tuple(v) if isinstance(v, list) else v for v in val))
        else:
            setattr(obj, key, val)


def load(path: str | None = None) -> Config:
    """Load configuration, falling back to the defaults above."""
    cfg = Config()
    path = path or DEFAULT_CONFIG
    if not os.path.exists(path):
        return cfg
    import yaml
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    limits_data = data.pop("limits", None)
    _apply(cfg, data, os.path.basename(path))
    if limits_data:
        profile = limits_data.pop("profile", "bench")
        cfg.limits = Limits.named(profile, **limits_data)
    cfg.source = path
    return cfg
