"""Companion-computer connector layer for a Betaflight-based airframe.

Modules, in dependency order:

    msp             MSP v1 client (framing, CRC, jumbo frames, thread-safe)
    safety          channel order (AETR) + the command envelope
    config          one place for ports, switch map, calibrations, limits
    math_utils      quaternion helpers (Hamilton, body FRD -> world NED)
    fc_link         FCLink: telemetry + safety-clamped override streaming
    imu_driver      attitude/IMU providers (FC-backed, fake, or dedicated)
    vision_adapter  tracker bbox -> world-frame bearing
    vision_interface  the Bearing type
    flight_logger   per-tick JSONL logging
    command_module  per-tick command producer (idle placeholder)

Bring-up harness: ``python3 -m tools.bringup --help``
"""

__version__ = "1.75.0"

from .config import Config, load  # noqa: F401
from .safety import Limits, aetr_frame  # noqa: F401
