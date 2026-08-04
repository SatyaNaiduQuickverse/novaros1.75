#!/usr/bin/env python3
"""Find a Betaflight flight controller on any port, at any baud.

    python3 tools/find_fc.py

Boards get swapped, and the by-id path, the baud, and even the transport
(USB VCP vs a GPIO UART) change with them. This probes every plausible port
with MSP_API_VERSION and reports what answered, so `config/vehicle.yaml` can be
pointed at the right thing instead of guessed at.

Read-only: it sends MSP requests and reads replies, and writes nothing to the
vehicle. Safe to run with a battery connected.
"""

from __future__ import annotations

import glob
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from companion.msp import (  # noqa: E402
    MSP, MSPError, BOX_MSP_OVERRIDE_PERMANENT_ID,
)

# USB VCP ignores baud entirely; real UARTs do not. 115200 is the Betaflight
# default, but MSP is often moved to a faster port.
BAUDS = (115200, 57600, 230400, 420000, 921600)


def candidate_ports() -> list[str]:
    ports = sorted(glob.glob("/dev/serial/by-id/*"))
    for pat in ("/dev/ttyACM*", "/dev/ttyUSB*", "/dev/ttyAMA*"):
        ports += sorted(glob.glob(pat))
    # /dev/serial0 is usually an alias for a ttyAMA already listed
    seen, out = set(), []
    for p in ports:
        real = os.path.realpath(p)
        if real in seen:
            continue
        seen.add(real)
        out.append(p)
    return out


def probe(port: str, baud: int, timeout: float = 0.35):
    try:
        m = MSP(port, baud, timeout=timeout)
    except Exception as e:
        return None, f"open failed: {e.__class__.__name__}"
    try:
        ident = m.identify()
        try:
            ids = m.box_ids()
            ident["override"] = BOX_MSP_OVERRIDE_PERMANENT_ID in ids
            ident["boxes"] = len(ids)
        except MSPError:
            ident["override"] = None
        return ident, None
    except MSPError:
        return None, "no MSP reply"
    except Exception as e:
        return None, f"{e.__class__.__name__}"
    finally:
        m.close()


def main():
    ports = candidate_ports()
    if not ports:
        print("no serial ports at all — is anything plugged in?")
        return 1
    print(f"probing {len(ports)} port(s) x {len(BAUDS)} baud rates\n")
    found = []
    for port in ports:
        real = os.path.realpath(port)
        label = port if port == real else f"{port} -> {real}"
        hits = []
        for baud in BAUDS:
            ident, err = probe(port, baud)
            if ident:
                hits.append((baud, ident))
                break  # a VCP answers at every baud; one hit is enough
        if hits:
            baud, ident = hits[0]
            ovr = ident.get("override")
            print(f"  FOUND  {label}")
            print(f"         {ident['variant']} {ident['firmware']} "
                  f"(MSP API {ident['msp_api']}) @ {baud}")
            print(f"         MSP OVERRIDE (box 50): "
                  f"{'present' if ovr else 'ABSENT' if ovr is False else 'unknown'}"
                  f"   boxes={ident.get('boxes', '?')}")
            found.append((port, baud, ident))
        else:
            print(f"  --     {label}: no MSP reply at any baud")
    print()
    if not found:
        print("No flight controller responded.")
        print("  * is the board powered (USB or battery)?")
        print("  * if it is on a GPIO UART, is MSP enabled on that UART in "
              "Betaflight, and are TX/RX crossed?")
        return 1
    port, baud, ident = found[0]
    print("point config/vehicle.yaml at it:")
    print(f"  fc:")
    print(f"    port: {port}")
    print(f"    baud: {baud}")
    if ident.get("override") is False:
        print("\nWARNING: this firmware has no MSP OVERRIDE (box id 50). "
              "Companion sticks will be ignored — flash a cloud build with the "
              "MSP Override option before any takeover work.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
