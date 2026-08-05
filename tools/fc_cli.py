#!/usr/bin/env python3
"""Read (and optionally set) a Betaflight CLI setting over the MSP port.

    python3 tools/fc_cli.py get msp_override_channels_mask
    python3 tools/fc_cli.py set msp_override_channels_mask 11     # writes EEPROM

Why the CLI and not MSP: Betaflight does NOT implement `MSP2_COMMON_SETTING`
(0x1003) — measured, it returns a well-formed `$X!` error frame, so the v2
transport is fine and the function simply is not there. That call is an INAV
extension. Configurator reads settings by dropping the same serial port into
CLI mode with a `#`, and so does this.

**This is the only code here that writes to the flight controller's EEPROM**,
and only on `set`. `get` never does. Both leave CLI mode, which reboots the FC
— harmless disarmed, and the reason this refuses to run while armed.

Verifying rather than assuming matters here: `channels.override_mask` in
vehicle.yaml must match the firmware, and a mismatch is not cosmetic. The wire
envelope check would police the wrong channels and could abort the stream on
the pilot's own throttle.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from companion.config import load                        # noqa: E402
from companion.msp import MSP, resolve_port              # noqa: E402

OK, BAD, WARN = "  [ OK ]", "  [FAIL]", "  [WARN]"


def _read_all(ser, secs=0.4):
    """Drain, tolerating the handle dying under us."""
    out, end = b"", time.time() + secs
    while time.time() < end:
        try:
            out += ser.read(ser.in_waiting or 1)
        except OSError:
            break
        time.sleep(0.05)
    return out.decode(errors="replace")


def cli_session(port, baud, lines, settle=0.45):
    """Enter CLI, run each line, return the transcript.

    MEASURED: on this board entering CLI mode reinitialises the USB VCP, so the
    handle open at the time dies with EIO mid-session. The port is therefore
    reopened after the `#` rather than assumed to survive it.
    """
    import serial
    ser = serial.Serial(port, baud, timeout=0.4, write_timeout=1.0)
    try:
        ser.reset_input_buffer()
        ser.write(b"#\r\n")
        ser.flush()
        time.sleep(0.4)
        out = _read_all(ser, 0.5)
    except OSError:
        out = ""
    finally:
        try:
            ser.close()
        except Exception:
            pass

    time.sleep(1.2)                      # let the VCP come back
    ser = serial.Serial(port, baud, timeout=0.4, write_timeout=1.0)
    try:
        ser.write(b"\r\n")
        ser.flush()
        out += _read_all(ser, 0.5)
        for ln in lines:
            ser.write(ln.encode() + b"\r\n")
            ser.flush()
            out += _read_all(ser, settle)
    finally:
        try:
            ser.close()
        except Exception:
            pass
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("action", choices=["get", "set"])
    ap.add_argument("name")
    ap.add_argument("value", nargs="?")
    a = ap.parse_args()

    cfg = load()
    port = resolve_port(cfg.fc.port)

    # Refuse while armed. Leaving CLI mode reboots the FC.
    m = MSP(port, cfg.fc.baud, cfg.fc.timeout_s)
    try:
        import struct
        from companion.msp import MSP_STATUS
        p = m.request(MSP_STATUS)
        flags = struct.unpack_from("<I", p, 6)[0]
        if flags & 1:
            print(BAD, "the FC is ARMED — refusing. Leaving CLI mode reboots it.")
            return 1
        print(f"fc: {m.identify()['variant']} {m.identify()['firmware']}  disarmed")
    finally:
        m.close()

    if a.action == "get":
        txt = cli_session(port, cfg.fc.baud, [f"get {a.name}", "exit"])
    else:
        if a.value is None:
            print(BAD, "set needs a value")
            return 1
        print(WARN, f"writing {a.name} = {a.value} to EEPROM, then rebooting")
        txt = cli_session(port, cfg.fc.baud,
                          [f"get {a.name}", f"set {a.name} = {a.value}", "save"],
                          settle=0.6)

    hits = [ln.strip() for ln in txt.splitlines() if a.name in ln]
    print()
    for ln in hits:
        print("   ", ln)
    if not hits:
        print(BAD, "no line mentioning that setting came back — full transcript:")
        print(txt[-600:])
        return 1
    print()
    print("The FC reboots on exit/save; give it ~3 s before reconnecting.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
