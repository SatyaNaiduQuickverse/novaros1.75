#!/usr/bin/env python3
"""Bind a config to the hardware it was measured on.

    python3 tools/bind_unit.py --id NOVA-002
    python3 tools/bind_unit.py --show

Records the connected FC's unique id and the ESP32's MAC into the config, so
the connectors can refuse to run against different hardware.

**Why this exists.** Every calibration in the config is per-unit: gyro bias and
accel offsets belong to a specific die, the mount matrix to a specific
mounting. Deploy one airframe's config to another and it inherits someone
else's idea of which way is down — and nothing downstream can tell. The numbers
are all plausible, `make test` passes, preflight is green, and the aircraft
flies wrong. Binding turns that silent failure into a refusal at connect.

Run it AFTER calibrating a unit, as the last step of commissioning.

Reads only, apart from writing the config. Never streams RC, never arms.
"""

from __future__ import annotations

import argparse
import os
import pathlib
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from companion.config import load, DEFAULT_CONFIG          # noqa: E402
from companion.imu_esp32 import ESP32IMU, find_esp32_port  # noqa: E402
from companion.msp import MSP, resolve_port                # noqa: E402

OK, BAD, WARN = "  [ OK ]", "  [FAIL]", "  [WARN]"


def read_hardware(cfg):
    """Identity of whatever is plugged in right now."""
    fc_id = esp_mac = ""
    try:
        m = MSP(resolve_port(cfg.fc.port), cfg.fc.baud, cfg.fc.timeout_s)
        try:
            fc_id = m.uid()
        finally:
            m.close()
    except Exception as e:
        print(WARN, f"could not read the FC: {e}")
    try:
        esp_mac = ESP32IMU.mac_from_port(find_esp32_port())
    except Exception as e:
        print(WARN, f"could not find the ESP32: {e}")
    return fc_id, esp_mac


def write_block(path, unit_id, fc_id, esp_mac):
    """Insert or update the `unit:` block, preserving every comment.

    Same reasoning as the rest of the config writers here: vehicle.yaml carries
    the measurement and date behind every value, and a PyYAML round-trip would
    delete all of it.
    """
    lines = pathlib.Path(path).read_text().splitlines(keepends=True)
    block = [
        "unit:\n",
        "  # Which physical airframe this config describes. Every calibration\n",
        "  # below is per-unit, so the connectors refuse to run against other\n",
        "  # hardware rather than fly a stranger's idea of which way is down.\n",
        "  # Written by tools/bind_unit.py — re-run it after any board swap.\n",
        f"  id: {unit_id}\n",
        f"  fc_mcu_id: {fc_id}\n",
        f"  esp32_mac: {esp_mac}\n",
        "  enforce: true\n",
        "\n",
    ]
    start = next((i for i, ln in enumerate(lines) if ln.rstrip() == "unit:"), None)
    if start is None:
        # Put it at the very top, after any leading comment banner, so the
        # first thing anyone reads is which aircraft this file is for.
        i = 0
        while i < len(lines) and (lines[i].startswith("#") or not lines[i].strip()):
            i += 1
        lines[i:i] = block
    else:
        end = start + 1
        while end < len(lines) and (lines[end].startswith((" ", "\t"))
                                    or not lines[end].strip()):
            end += 1
        lines[start:end] = block
    pathlib.Path(path).write_text("".join(lines))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--id", help="human label for this unit, e.g. NOVA-002")
    ap.add_argument("--show", action="store_true", help="compare, change nothing")
    ap.add_argument("--config", default=None)
    a = ap.parse_args()

    cfg = load(a.config)
    path = a.config or cfg.source or DEFAULT_CONFIG
    fc_id, esp_mac = read_hardware(cfg)

    print(f"config: {path}")
    print(f"  recorded : id={cfg.unit.id or '-'}  fc={cfg.unit.fc_mcu_id or '-'}  "
          f"esp32={cfg.unit.esp32_mac or '-'}")
    print(f"  connected: {'':9} fc={fc_id or '-'}  esp32={esp_mac or '-'}")
    print()

    for label, want, got in (("FC", cfg.unit.fc_mcu_id, fc_id),
                             ("ESP32", cfg.unit.esp32_mac, esp_mac)):
        if not want:
            print(WARN, f"{label}: not bound")
        elif not got:
            print(WARN, f"{label}: not connected, cannot compare")
        elif want.lower() == got.lower():
            print(OK, f"{label}: matches")
        else:
            print(BAD, f"{label}: MISMATCH — this config was measured on other "
                       f"hardware, so its calibration does not belong here")

    if a.show:
        return 0
    if not a.id and not cfg.unit.id:
        print("\n" + BAD, "--id is required the first time (e.g. --id NOVA-002)")
        return 1
    if not (fc_id or esp_mac):
        print("\n" + BAD, "neither board is connected — nothing to bind to")
        return 1

    unit_id = a.id or cfg.unit.id
    write_block(path, unit_id, fc_id, esp_mac)
    print(f"\n  bound {path} to unit {unit_id}")
    print("  Commit it. A config that is not bound cannot tell whether its")
    print("  calibration belongs to the airframe it is running on.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
