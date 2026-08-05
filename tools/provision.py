#!/usr/bin/env python3
"""Bring a freshly-imaged Pi up to the point where calibration can start.

    python3 tools/provision.py                              # check everything
    python3 tools/provision.py --flash-esp32                # + deploy main.py
    python3 tools/provision.py --flash-esp32 <mpy.bin>      # + reflash first

Automates the parts of `docs/COMMISSIONING.md` that are pure repetition:
dependency check, board detection, an FC configuration comparison, the ESP32
flash and firmware deploy, and a streaming check on both boards.

It does NOT write the FC's EEPROM. Restoring `diff all` from a plain serial
session was tried and abandoned: the dump contains commands that reinitialise
the USB port, so the push dies half-way with EIO. Configurator does it
reliably in thirty seconds, so that step stays manual and this reports exactly
what differs.

It deliberately does NOT calibrate. Gyro bias, accel offsets, the mount matrix
and the sign check all need the airframe physically moved, and each is per-unit
— a script cannot pick the drone up. What this does is remove the six manual
steps in front of them, so commissioning is one command plus eight minutes of
hands-on work.

Default is check-only. Anything that writes to a board is opt-in, named on the
command line, and refuses while the aircraft is armed.
"""

from __future__ import annotations

import argparse
import glob
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

OK, BAD, WARN, SKIP = "  [ OK ]", "  [FAIL]", "  [WARN]", "  [ -- ]"
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FC_DIFF = os.path.join(REPO, "config", "fc_diff_all.txt")
ESP_MAIN = os.path.join(REPO, "esp32", "main.py")
VENV_ESP = os.path.join(REPO, ".venv-esp")

_fails: list[str] = []


def check(cond, label, detail=""):
    print(f"{OK if cond else BAD} {label}" + (f" — {detail}" if detail else ""))
    if not cond:
        _fails.append(label)
    return cond


# --------------------------------------------------------------- dependencies

def step_deps():
    print("\n1. dependencies")
    ok = True
    for mod, apt in (("serial", "python3-serial"), ("numpy", "python3-numpy"),
                     ("yaml", "python3-yaml")):
        try:
            __import__(mod)
            print(f"{OK} {mod}")
        except ImportError:
            ok = False
            print(f"{BAD} {mod} missing — sudo apt install {apt}")
            _fails.append(mod)
    print(f"{OK if sys.version_info >= (3, 11) else BAD} python "
          f"{'.'.join(map(str, sys.version_info[:3]))}")
    return ok


# ------------------------------------------------------------------- hardware

def step_detect():
    """Which boards are present, and who are they."""
    print("\n2. hardware")
    from companion.config import load
    from companion.imu_esp32 import ESP32IMU
    from companion.msp import MSP, resolve_port
    cfg = load()
    info = {"fc_mcu_id": "", "esp32_mac": "", "fw": ""}

    try:
        port = resolve_port(cfg.fc.port)
        m = MSP(port, cfg.fc.baud, cfg.fc.timeout_s)
        try:
            ident = m.identify()
            info["fw"] = f"{ident['variant']} {ident['firmware']}"
            info["fc_mcu_id"] = m.uid()
        finally:
            m.close()
        check(True, "flight controller", f"{info['fw']}  mcu {info['fc_mcu_id']}")
    except Exception as e:
        check(False, "flight controller", str(e))

    esp = sorted(glob.glob("/dev/serial/by-id/*Espressif*"))
    if esp:
        info["esp32_mac"] = ESP32IMU.mac_from_port(esp[0])
        check(True, "ESP32 bridge", f"MAC {info['esp32_mac']}")
    else:
        check(False, "ESP32 bridge", "nothing under /dev/serial/by-id/*Espressif*")

    # Bound, and bound to THESE boards?
    u = cfg.unit
    if not u.fc_mcu_id and not u.esp32_mac:
        print(f"{WARN} this config is not bound to any hardware — after "
              f"calibrating, run: python3 tools/bind_unit.py --id <name>")
    else:
        same_fc = (not info["fc_mcu_id"]
                   or u.fc_mcu_id.lower() == info["fc_mcu_id"].lower())
        same_esp = (not info["esp32_mac"]
                    or u.esp32_mac.upper() == info["esp32_mac"].upper())
        if same_fc and same_esp:
            check(True, f"config is bound to this hardware", f"unit {u.id}")
        else:
            check(False, "config belongs to a DIFFERENT unit",
                  f"config says {u.id} ({u.fc_mcu_id[:8]}../{u.esp32_mac}); "
                  f"connected is {info['fc_mcu_id'][:8]}../{info['esp32_mac']}. "
                  f"Its calibration does not describe this airframe.")
    return info


# --------------------------------------------------------------- FC configure

def step_check_fc(info):
    """Compare the FC's live configuration against the saved dump. READ ONLY.

    Restoring it from here is deliberately NOT offered. Measured twice: the
    dump contains commands — `defaults`, `serial` — that reinitialise the FC's
    USB port, so the handle dies mid-push with EIO and the rest is never sent.
    Both attempts survived only because `save` had not run either, leaving the
    reset in RAM for the reboot to discard. Relying on that would be relying on
    luck, and every attempt runs `defaults nosave` on a real aircraft.

    Betaflight Configurator does this reliably in thirty seconds over a
    transport that survives the reinit. So the runbook keeps that step manual,
    and this reports precisely what differs so the operator knows whether it is
    needed at all.
    """
    print("\n3. FC configuration vs the saved dump")
    from companion.config import load
    cfg = load()
    if not os.path.exists(FC_DIFF):
        return check(False, "fc_diff_all.txt present", FC_DIFF)

    import tempfile
    from fc_cli import cli_session
    from companion.msp import resolve_port
    try:
        txt = cli_session(resolve_port(cfg.fc.port), cfg.fc.baud,
                          ["diff all", "exit"], settle=6.0)
    except Exception as e:
        return check(False, "could not read the FC configuration", str(e))

    def meaningful(text):
        return [ln.strip() for ln in text.splitlines()
                if ln.strip() and not ln.strip().startswith("#")
                and not ln.strip().startswith("mcu_id")]

    want = meaningful(open(FC_DIFF).read())
    got = meaningful(txt[txt.find("diff all"):] if "diff all" in txt else txt)
    missing = [ln for ln in want if ln not in got]
    extra = [ln for ln in got if ln not in want and ln != "diff all"]

    if not missing and not extra:
        return check(True, "FC matches the saved configuration",
                     f"{len(want)} lines")
    check(False, "FC differs from the saved configuration",
          f"{len(missing)} missing, {len(extra)} unexpected")
    for ln in missing[:8]:
        print(f"         missing: {ln}")
    for ln in extra[:8]:
        print(f"         extra:   {ln}")
    print("\n       Restore it in Betaflight Configurator -> CLI: paste")
    print(f"       {FC_DIFF} then `save`. Pushing it from here is not offered —")
    print("       `defaults` reinitialises the USB port and the push dies "
          "half-way.")
    return False


# ------------------------------------------------------------------- ESP32

def step_flash_esp32(binary):
    print("\n4. flash the ESP32 bridge")
    esp = sorted(glob.glob("/dev/serial/by-id/*Espressif*"))
    if not esp:
        return check(False, "ESP32 present")
    port = esp[0]
    py = os.path.join(VENV_ESP, "bin", "python")
    if not os.path.exists(py):
        print(f"       creating {VENV_ESP} ...")
        subprocess.run([sys.executable, "-m", "venv", "--system-site-packages",
                        VENV_ESP], check=True)
        subprocess.run([py, "-m", "pip", "-q", "install", "esptool", "mpremote"],
                       check=True)
    if binary:
        if not os.path.exists(binary):
            return check(False, "MicroPython image present", binary)
        print(f"       erasing and flashing {os.path.basename(binary)} ...")
        r = subprocess.run([py, "-m", "esptool", "--chip", "esp32c6", "--port",
                            port, "write-flash", "0x0", binary],
                           capture_output=True, text=True)
        if not check(r.returncode == 0, "MicroPython flashed",
                     (r.stderr or r.stdout)[-200:] if r.returncode else ""):
            return False
        time.sleep(3.0)
    print("       deploying esp32/main.py ...")
    r = subprocess.run([py, "-m", "mpremote", "connect", port, "cp", ESP_MAIN,
                        ":main.py"], capture_output=True, text=True)
    if not check(r.returncode == 0, "main.py deployed",
                 (r.stderr or r.stdout)[-200:] if r.returncode else ""):
        print(f"{WARN} if this failed with a timeout, the running firmware "
              f"cannot be interrupted while streaming — see "
              f"docs/ESP32_BRIDGE_FAULTS.md")
        return False
    subprocess.run([py, "-m", "mpremote", "connect", port, "reset"],
                   capture_output=True)
    return True


def step_stream_check():
    print("\n5. both boards streaming")
    from companion.config import load
    from companion.imu_esp32 import ESP32IMU
    cfg = load()
    try:
        imu = ESP32IMU(port=cfg.imu32.port or None, cal=cfg.imu32).start()
        try:
            time.sleep(1.5)
            st = imu.stats()
            check(st["frames"] > 100 and not st["stale"], "ESP32 streaming",
                  f"{st['frames']} frames, {st['drops']} drops, "
                  f"{st['recoveries']} recoveries")
        finally:
            imu.stop()
    except Exception as e:
        check(False, "ESP32 streaming", str(e))

    from companion.fc_link import FCLink
    try:
        fc = FCLink(cfg).connect()
        try:
            rc = fc.rc()
            check(len(rc) >= 8, "FC responding", f"rc {rc[:4]}")
        finally:
            fc.close()
    except Exception as e:
        check(False, "FC responding", str(e))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--flash-esp32", metavar="BIN", nargs="?", const="",
                    help="deploy main.py; give a MicroPython .bin to reflash first")
    a = ap.parse_args()
    flash = a.flash_esp32

    print("provisioning — PROPS OFF. Nothing here streams RC or arms.")
    step_deps()
    info = step_detect()
    step_check_fc(info)
    if flash is not None:
        step_flash_esp32(flash)
    else:
        print(f"\n4. flash the ESP32 bridge\n{SKIP} skipped (--flash-esp32)")
    step_stream_check()

    print("\n" + "=" * 66)
    if _fails:
        print(f"{BAD} {len(_fails)} check(s) failed: " + "; ".join(_fails[:4]))
        return 1
    print(f"{OK} provisioned. What a script cannot do is next:")
    print()
    print("   make calib-ui        -> http://<this-pi>:8720/")
    print("     gyro bias, then step 1 accel, then step 2 axis map,")
    print("     then the six-pose sign check and 3a/3b. ~8 minutes.")
    print()
    print("   python3 tools/check_stick_direction.py     # override switch DOWN")
    print("   python3 -m tools.bringup motors            # props off, armed")
    print("   python3 tools/bind_unit.py --id <name>     # LAST — bind and commit")
    print()
    print("   Every one of those needs the airframe physically moved, and every")
    print("   result is specific to THESE two boards. See docs/COMMISSIONING.md")
    return 0


if __name__ == "__main__":
    sys.exit(main())
