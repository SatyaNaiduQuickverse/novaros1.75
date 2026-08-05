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

import pathlib                                            # noqa: E402

from companion.config import load, DEFAULT_CONFIG         # noqa: E402
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


DUMP_PATH = "config/fc_diff_all.txt"


def _dump(a):
    """Save the FC's entire configuration to the repo.

    Everything that makes this airframe work lives in the flight controller's
    EEPROM and NOWHERE ELSE: the override mask, the aux bindings that give ch5
    / ch8 / ch9 any meaning, PIDs, rates, motor order, the accelerometer
    calibration. The repo reproduces the software exactly and could not
    reproduce any of that. Reset the board or swap it and it is simply gone.

    `diff all` is Betaflight's own restore format — paste it back into the CLI
    and the board returns to this state. Committing it makes the FC's
    configuration version-controlled alongside the code that assumes it.
    """
    cfg = load()
    out = a.name or DUMP_PATH
    print(f"reading `diff all` from the FC (this takes a few seconds) ...")
    txt = cli_session(resolve_port(cfg.fc.port), cfg.fc.baud,
                      ["diff all", "exit"], settle=6.0)
    # Keep from the command echo to the end of the dump, dropping the CLI
    # banner and our own prompts.
    start = txt.find("# diff all")
    if start < 0:
        start = txt.find("diff all")
    body = txt[start:] if start >= 0 else txt
    body = body.replace("\r\n", "\n")
    lines = [ln for ln in body.splitlines() if not ln.strip().startswith("#$")]
    if len(lines) < 20:
        print(BAD, f"only {len(lines)} lines came back — the dump looks truncated")
        print(txt[-400:])
        return 1
    pathlib.Path(out).write_text("\n".join(lines).rstrip() + "\n")
    n_set = sum(1 for ln in lines if ln.strip().startswith("set "))
    n_aux = sum(1 for ln in lines if ln.strip().startswith("aux "))
    print(f"\n  wrote {out}")
    print(f"  {len(lines)} lines, {n_set} `set` values, {n_aux} aux bindings")
    for ln in lines:
        t = ln.strip()
        if t.startswith("aux ") or "msp_override_channels_mask" in t:
            print("   ", t)
    print("\n  Restore by pasting the file into Betaflight Configurator's CLI,")
    print("  then `save`. Commit it so the FC's config is versioned with the")
    print("  code that assumes it.")
    return 0


def _readback_mask(cfg):
    """What the FC says the mask is, right now. Source of truth."""
    port = resolve_port(cfg.fc.port)
    txt = cli_session(port, cfg.fc.baud, ["get msp_override_channels_mask", "exit"])
    for ln in txt.splitlines():
        ln = ln.strip()
        if ln.startswith("msp_override_channels_mask") and "=" in ln:
            return int(ln.split("=")[1].strip())
    raise RuntimeError("the FC did not report the mask back")


def _set_mask(a):
    cfg = load()
    want = int(a.value)
    print(WARN, f"writing msp_override_channels_mask = {want}, then rebooting")
    cli_session(resolve_port(cfg.fc.port), cfg.fc.baud,
                [f"set msp_override_channels_mask = {want}", "save"], settle=0.6)
    time.sleep(4.0)                       # the save reboots the FC
    got = _readback_mask(cfg)
    print(f"\n  FC reports: {got}  (0b{got:b})")
    if got != want:
        print(BAD, f"the FC has {got}, not {want} — config NOT touched")
        return 1

    path = cfg.source or DEFAULT_CONFIG
    lines = pathlib.Path(path).read_text().splitlines(keepends=True)
    hit = False
    for i, ln in enumerate(lines):
        if ln.lstrip().startswith("override_mask:") and not ln.lstrip().startswith("#"):
            lines[i] = f"  override_mask: {got}\n"
            hit = True
            break
    if not hit:
        print(BAD, f"no override_mask line in {path} — set it by hand to {got}")
        return 1
    pathlib.Path(path).write_text("".join(lines))
    print(f"  {path}: override_mask -> {got}")

    bits = {0: "ch1 roll", 1: "ch2 pitch", 2: "ch3 throttle", 3: "ch4 yaw",
            8: "ch9 ARM"}
    print()
    for b, nm in bits.items():
        print("   %-14s %s" % (nm, "COMPANION" if got >> b & 1 else "pilot"))
    if not (got & (1 << 2)):
        print("\n  " + OK.strip() + " the pilot keeps throttle — chopping it is an "
              "always-works cutout")
    else:
        print("\n  " + WARN.strip() + " the companion owns throttle: the pilot's "
              "stick is INERT and the")
        print("         override switch is the only cutout. Command throttle "
              "every tick.")
    if load().channels.companion_arm and not (got & (1 << 8)):
        print("  " + WARN.strip() + " companion_arm is set but ch9 is not in the "
              "mask — arm() will refuse.")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("action", choices=["get", "set", "set-mask", "dump"])
    ap.add_argument("name", nargs="?")
    ap.add_argument("value", nargs="?")
    a = ap.parse_args()
    if a.action == "dump":
        return _dump(a)
    if a.action == "set-mask":
        # One value, one truth. The mask lives on the FC AND in vehicle.yaml,
        # and a mismatch is not cosmetic: the wire watchdog uses the config
        # copy to decide which channels to police, so a stale value makes it
        # judge the PILOT's throttle against the companion's cap and abort the
        # stream the moment they advance it. This writes the FC, reads the FC
        # BACK, and derives the config from what the hardware actually reports
        # — so the two cannot drift, because one is no longer typed alongside
        # the other.
        if a.name is None:
            print(BAD, "set-mask needs a value, e.g. 15")
            return 1
        a.value, a.name = a.name, "msp_override_channels_mask"
        a.action = "set"
        return _set_mask(a)

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
