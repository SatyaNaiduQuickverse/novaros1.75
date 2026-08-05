#!/usr/bin/env python3
"""Bench harness: bring up and VERIFY each connector, one at a time.

Run in order and get each step green before starting the next.

    python3 -m tools.bringup preflight               # who's on the port, what's configured
    python3 -m tools.bringup link                    # telemetry from the FC
    python3 -m tools.bringup calib-imu               # measure acc scale + axis map
    python3 -m tools.bringup imu        [--fake]     # (q, w, accel) contract
    python3 -m tools.bringup vision     [--fake]     # a bearing from the tracker
    python3 -m tools.bringup rc                      # stream idle RC   (PROPS OFF)
    python3 -m tools.bringup authority               # measure who owns ch1-4
    python3 -m tools.bringup override                # guided takeover + handback test
    python3 -m tools.bringup loop  [--fake-imu --fake-vision --no-fc]

PROPS OFF for every step here. A pilot with a transmitter must be able to take
control instantly at all times — that is the safety layer these tests run behind.
"""

from __future__ import annotations

import argparse
import logging
import math
import os
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from companion.config import load as load_config          # noqa: E402
from companion.fc_link import FCLink, decode_raw_imu       # noqa: E402
from companion.flight_logger import FlightLogger           # noqa: E402
from companion.imu_driver import FakeIMU, make_imu         # noqa: E402
from companion.msp import MSP_RAW_IMU, MSPError, resolve_port  # noqa: E402
from companion.safety import IDLE_STICKS                   # noqa: E402
from companion.vision_adapter import VisionAdapter         # noqa: E402

G = 9.80665
OK, BAD, WARN = "  [ OK ]", "  [FAIL]", "  [WARN]"


def _connect(a, **kw) -> FCLink:
    cfg = load_config(a.config)
    return FCLink(cfg, port=a.port, **kw).connect()


def _fake_or(a, attr="fake"):
    return getattr(a, attr, False)


# --------------------------------------------------------------- recording

class BenchRecorder:
    """Per-test JSONL record, so a bench run can be analysed after the fact.

    The interactive tests need a human at the transmitter, so their result
    otherwise exists only as text on someone's terminal. This samples FC state
    on a background thread and timestamps every prompt and verdict, which is
    what makes latencies measurable rather than eyeballed.

    Sampling costs MSP transactions from the same ~99/s budget as the RC
    stream, so it is deliberately slow: state at 5 Hz (1 txn each) and the
    heavier rc/motor reads at 1 Hz.
    """

    def __init__(self, cmd, fc=None, run_dir="~/logs", state_hz=5.0, deep_hz=1.0):
        stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        self.lg = FlightLogger(path=os.path.join(
            os.path.expanduser(run_dir), f"bench_{cmd}_{stamp}.jsonl"))
        self.cmd = cmd
        self.fc = fc
        self._stop = threading.Event()
        self._th = None
        self.state_hz, self.deep_hz = state_hz, deep_hz
        self.lg.event("bench_start", cmd=cmd)

    def start_sampling(self):
        if self.fc is None:
            return self
        period = 1.0 / self.state_hz
        every = max(int(self.state_hz / self.deep_hz), 1)

        def loop():
            n = 0
            while not self._stop.is_set():
                t0 = time.perf_counter()
                try:
                    flags, adf = self.fc._status()
                    rec = {
                        "armed": bool(flags & (1 << self.fc.arm_bit)),
                        "override": (self.fc.override_bit is not None
                                     and bool(flags & (1 << self.fc.override_bit))),
                        "arming_disable": adf,
                    }
                    if n % every == 0:
                        rec["rc"] = self.fc.rc()
                        rec["motors"] = self.fc.motors()
                    self.lg.log(sample=1, **rec)
                except Exception as e:
                    self.lg.log(sample=1, error=repr(e))
                n += 1
                slp = period - (time.perf_counter() - t0)
                if slp > 0:
                    time.sleep(slp)

        self._th = threading.Thread(target=loop, name="bench-sampler", daemon=True)
        self._th.start()
        return self

    def event(self, name, **kw):
        print_kw = " ".join(f"{k}={v}" for k, v in kw.items())
        self.lg.event(name, **kw)
        return f"{name} {print_kw}".strip()

    def close(self, **summary):
        self._stop.set()
        if self._th:
            self._th.join(timeout=1.0)
        if self.fc is not None:
            try:
                summary.setdefault("crc_errors", self.fc.msp.crc_errors)
                summary.setdefault("timeouts", self.fc.msp.timeouts)
            except Exception:
                pass
        self.lg.event("bench_end", **summary)
        self.lg.close()
        print(f"\nrecord: {self.lg.path}")
        print(f"analyse: python3 tools/log_analyze.py {self.lg.path}")
        return self.lg.path


# ---------------------------------------------------------------- preflight

def t_preflight(a):
    cfg = load_config(a.config)
    print(f"config   : {cfg.source}")
    print(f"limits   : profile={cfg.limits.profile} thr<={cfg.limits.thr_cap} "
          f"deflect<=±{cfg.limits.max_deflect} motor_abort={cfg.limits.motor_abort}")
    try:
        port = resolve_port(a.port or cfg.fc.port)
        print(f"port     : {port}")
    except MSPError as e:
        print(BAD, e)
        return 1

    fc = FCLink(cfg, port=a.port).connect()
    try:
        ident = fc.msp.identify()
        print(f"firmware : {ident['variant']} {ident['firmware']} (MSP API {ident['msp_api']})")
        print(OK, "MSP OVERRIDE present (box id 50)")
        rc = fc.rc()
        ch = cfg.channels
        print(f"rc       : {rc}")
        print(f"  ch{ch.override_index + 1} override switch = "
              f"{rc[ch.override_index] if len(rc) > ch.override_index else '?'} us")
        print(f"  ch{ch.arm_index + 1} ARM switch      = "
              f"{rc[ch.arm_index] if len(rc) > ch.arm_index else '?'} us  (pilot only)")
        # Read the aux bindings the FC actually has saved, rather than trusting
        # the config. A board swap or a re-bind otherwise leaves the operator
        # believing a switch disengages the companion when it does not.
        try:
            ranges = fc.mode_ranges()
            print("  aux bindings saved on the FC:")
            found = {}
            for mr in sorted(ranges, key=lambda x: x["channel"]):
                now = rc[mr["index"]] if len(rc) > mr["index"] else None
                act = "ACTIVE" if now is not None and mr["lo"] <= now <= mr["hi"] else ""
                print(f"    {mr['name']:<14} AUX{mr['aux']}/ch{mr['channel']:<3} "
                      f"{mr['lo']}-{mr['hi']:<5} now={now} {act}")
                found[mr["name"]] = mr["index"]
            for label, name, want in (("override", "MSP OVERRIDE", ch.override_index),
                                      ("ARM", "ARM", ch.arm_index)):
                got = found.get(name)
                if got is None:
                    print(BAD, f"{name} is not bound to any switch on this FC")
                elif got != want:
                    print(BAD, f"{label} index mismatch: FC has ch{got + 1}, "
                               f"config says ch{want + 1} — fix channels."
                               f"{'override_index' if label == 'override' else 'arm_index'}")
                else:
                    print(OK, f"{label} switch matches config (ch{want + 1})")
        except MSPError as e:
            print(WARN, f"could not read mode ranges: {e}")
        b = fc.txn_budget()
        print(f"  MSP budget      : {b['demand']:.0f}/{b['ceiling']:.0f} txn/s "
              f"({100 * b['load']:.0f}%)"
              + ("  <-- OVERSUBSCRIBED" if b["load"] > 0.9 else ""))
        print((OK if fc.rx_link_up() else BAD) + " pilot RC link")
        if not fc.rx_link_up():
            print("         ELRS receiver in WiFi mode outputs no CRSF — power-cycle "
                  "the battery AND USB with the transmitter already on")
        print((WARN if fc.armed() else OK) + f" armed={fc.armed()}"
              + ("  <-- DISARM before bench work" if fc.armed() else ""))
        print(f"  override active : {fc.override_active()}")
        print(f"  motors          : {fc.motors()}")
        print(f"  attitude        : {fc.attitude()}")
        pending = cfg.unverified()
        if pending:
            print("\nunverified calibrations (each silently biases everything downstream):")
            for p in pending:
                print("  -", p)
        else:
            print(OK, "all calibrations marked verified")
        print(f"\ncrc errors: {fc.msp.crc_errors}")
    finally:
        fc.close()
    return 0


# ------------------------------------------------------------------- link

def t_link(a):
    fc = _connect(a)
    fc.start_telemetry()
    print("reading telemetry for 5 s ...")
    t0 = time.time()
    while time.time() - t0 < 5:
        att, imu = fc.read_attitude(), fc.read_imu()
        if att and imu:
            acc, gyro = imu
            print(f"  roll={att['roll']:+7.1f} pitch={att['pitch']:+7.1f} "
                  f"yaw={att['yaw']:+6.0f} | acc={np.round(acc, 2)} "
                  f"gyro={np.round(gyro, 3)}")
        time.sleep(0.5)
    print(f"crc errors: {fc.msp.crc_errors}, timeouts: {fc.msp.timeouts}")
    fc.close()
    print("LINK OK" if fc.msp.crc_errors == 0 else "LINK UP but CRC errors seen")


# --------------------------------------------------------------- calib-imu

def _sample_raw(fc, n=40):
    """Average n raw MSP_RAW_IMU reads. Returns board-frame ints."""
    import struct
    acc, gyro = np.zeros(3), np.zeros(3)
    got = 0
    for _ in range(n):
        try:
            p = fc.msp.request(MSP_RAW_IMU)
        except MSPError:
            continue
        v = struct.unpack_from("<6h", p, 0)
        acc += np.array(v[0:3], float)
        gyro += np.array(v[3:6], float)
        got += 1
        time.sleep(0.01)
    if not got:
        raise MSPError("no IMU samples")
    return acc / got, gyro / got


def _prompt(msg):
    try:
        input(f"\n>>> {msg}\n    press ENTER when ready (ctrl-C to stop) ")
    except (EOFError, KeyboardInterrupt):
        raise SystemExit("\ncalibration aborted")


def t_calib_imu(a):
    """Measure the accelerometer scale and the board->FRD axis map.

    Betaflight's MSP_RAW_IMU units and sensor-frame orientation vary by build
    and by board, and a wrong map is invisible downstream — it just points every
    bearing slightly the wrong way. So measure it here rather than assume.
    """
    fc = _connect(a)
    cal = fc.imu_cal
    rec = BenchRecorder("calib", fc, run_dir=load_config(a.config).log_dir)
    print("PROPS OFF. This needs you to physically move the airframe.")

    _prompt("Put the airframe LEVEL and completely STILL.")
    acc_raw, gyro_raw = _sample_raw(fc)
    rec.event("pose_level", acc_raw=list(acc_raw), gyro_raw=list(gyro_raw),
              attitude=fc.attitude())
    mag = float(np.linalg.norm(acc_raw))
    grav_axis = int(np.argmax(np.abs(acc_raw)))
    grav_sign = float(np.sign(acc_raw[grav_axis]))
    print(f"    raw acc  = {np.round(acc_raw, 1)}  |acc| = {mag:.1f}")
    print(f"    raw gyro = {np.round(gyro_raw, 2)}  (should be ~0 at rest)")
    print(f"    -> acc_per_g = {mag:.1f}   (configured: {cal.acc_per_g})")
    print(f"    -> gravity is on board axis {grav_axis}, sign {grav_sign:+.0f}")
    if abs(gyro_raw).max() > 5:
        print(WARN, "gyro is not near zero at rest — was it moving, or is the "
                    "gyro uncalibrated? Run a Betaflight gyro cal.")

    # A level airframe must read -1 g on FRD z (specific force points up).
    zmap = (grav_axis, -grav_sign)

    _prompt("Tilt the NOSE UP about 30 degrees and hold it still.")
    acc_up, _ = _sample_raw(fc, 25)
    att_up = fc.attitude()
    rec.event("pose_nose_up", acc_raw=list(acc_up), attitude=att_up)
    delta_up = acc_up - acc_raw
    x_axis = int(np.argmax(np.abs(np.delete(delta_up, grav_axis))))
    x_axis = [i for i in range(3) if i != grav_axis][x_axis]
    # nose up => FRD x specific force goes POSITIVE
    xmap = (x_axis, float(np.sign(delta_up[x_axis])))
    pitch_sign = 1.0 if att_up["pitch"] > 0 else -1.0
    print(f"    raw acc = {np.round(acc_up, 1)}  FC pitch = {att_up['pitch']:+.1f} deg")

    _prompt("Roll the airframe RIGHT (right side down) about 30 degrees, hold still.")
    acc_ri, _ = _sample_raw(fc, 25)
    att_ri = fc.attitude()
    rec.event("pose_roll_right", acc_raw=list(acc_ri), attitude=att_ri)
    delta_ri = acc_ri - acc_raw
    remaining = [i for i in range(3) if i not in (grav_axis, x_axis)]
    y_axis = remaining[0]
    # roll right => FRD y specific force goes NEGATIVE
    ymap = (y_axis, -float(np.sign(delta_ri[y_axis])))
    roll_sign = 1.0 if att_ri["roll"] > 0 else -1.0
    print(f"    raw acc = {np.round(acc_ri, 1)}  FC roll = {att_ri['roll']:+.1f} deg")

    board_to_frd = (xmap, ymap, zmap)
    print("\n" + "=" * 72)
    print("measured calibration — paste into config/vehicle.yaml:\n")
    print("imu:")
    print(f"  acc_per_g: {mag:.0f}")
    print(f"  gyro_units: {cal.gyro_units}")
    print("  board_to_frd:")
    for src, sign in board_to_frd:
        print(f"    - [{src}, {sign:+.0f}]")
    print(f"  roll_sign: {roll_sign:+.0f}")
    print(f"  pitch_sign: {pitch_sign:+.0f}")
    print(f"  yaw_sign: {cal.yaw_sign:+.0f}")
    print("  verified: true")
    print("=" * 72)

    # Verify by re-decoding the level sample through the measured map.
    class _Tmp:
        acc_per_g = mag
        gyro_units = cal.gyro_units
        board_to_frd = tuple(board_to_frd)
    acc_frd, _ = decode_raw_imu([*acc_raw.astype(int), 0, 0, 0], _Tmp)
    print(f"\nlevel sample through the measured map: {np.round(acc_frd, 2)} m/s^2")
    good = abs(acc_frd[2] + G) < 0.6 and np.linalg.norm(acc_frd[:2]) < 1.0
    print((OK if good else BAD) + " expected ~[0, 0, -9.81] (body FRD)")
    if not good:
        print("       re-run on a genuinely level, still surface before trusting it")
    rec.event("calib_result", acc_per_g=round(mag, 1),
              board_to_frd=[list(x) for x in board_to_frd],
              roll_sign=roll_sign, pitch_sign=pitch_sign,
              level_check_frd=list(acc_frd), passed=bool(good))
    rec.close(passed=bool(good))
    fc.close()
    return 0 if good else 1


# -------------------------------------------------------------------- imu

def t_imu(a):
    fc = None
    if a.fake:
        imu = FakeIMU()
        src = "FAKE"
    else:
        fc = _connect(a)
        fc.start_telemetry()
        imu = make_imu(a.imu, fc=fc, cfg=load_config(a.config))
        src = a.imu.upper()
    imu.start()
    print(f"IMU ({src}) — get_state() for 5 s ...")
    t0 = time.time()
    while time.time() - t0 < 5:
        q, w, acc = imu.get_state()
        print(f"  q={np.round(q, 3)} w={np.round(w, 3)} acc={np.round(acc, 2)}")
        time.sleep(0.5)
    imu.stop()
    if fc:
        fc.close()
    print("check: q tracks hand-tilts; level -> acc ~[0,0,-9.8] FRD; axes correct.")


# ----------------------------------------------------------------- vision

def t_vision(a):
    cfg = load_config(a.config)
    fc = None
    if a.fake:
        attitude = lambda: np.array([1.0, 0, 0, 0])
    else:
        fc = _connect(a)
        fc.start_telemetry()
        attitude = fc.attitude_q
    bbox_src = None
    if a.tracker_url:
        from companion.vision_adapter import http_bbox_source
        bbox_src = http_bbox_source(a.tracker_url)
    va = VisionAdapter(cfg.camera, get_bbox_fn=bbox_src,
                       get_attitude_fn=attitude, fake=a.fake)
    print(f"VISION ({'FAKE' if a.fake else 'REAL'}) — bearing() x10 ...")
    hits = 0
    for _ in range(10):
        b = va.bearing(time.monotonic())
        if b:
            hits += 1
            print(f"  {b}")
        else:
            print("  no measurement")
        time.sleep(0.3)
    if fc:
        fc.close()
    print(f"{hits}/10 measurements.")
    if not cfg.camera.verified:
        print(WARN, "camera intrinsics/boresight are still the defaults — "
                    "calibrate before trusting any bearing")
    print("check: the bearing points the right way as the object and vehicle move.")


# --------------------------------------------------------------------- rc

def t_rc(a):
    print("!! PROPS OFF. Streaming IDLE RC (centred, throttle floor) for 5 s ...")
    fc = _connect(a)
    fc.start_rc_stream()
    t0 = time.time()
    while time.time() - t0 < 5 and not fc.abort_reason:
        time.sleep(0.25)
        print(f"  sent={fc.frames_sent} override={fc.override_active()} "
              f"rc={fc.rc()[:4]}")
    print(f"frames sent: {fc.frames_sent}  abort: {fc.abort_reason}")
    fc.close()
    print("Confirm the FC saw RC in the Betaflight receiver tab, and that the "
          "pilot transmitter can take control at any moment.")


# --------------------------------------------------------------- override

def t_override(a):
    """Guided takeover + handback test. Disarmed, props off.

    Proves the three things that matter before any powered work: the companion
    can take the sticks, the pilot can take them back, and stopping the stream
    alone hands control back.
    """
    cfg = load_config(a.config)
    fc = _connect(a)
    ch = cfg.channels
    rec = BenchRecorder("override", fc, run_dir=cfg.log_dir).start_sampling()
    try:
        if fc.armed():
            print(BAD, "vehicle is ARMED — disarm before this test")
            return 1
        if not fc.rx_link_up():
            print(BAD, "pilot RC link is down — transmitter must be on and linked")
            return 1
        if fc.override_active():
            print(BAD, f"override is ALREADY active (ch{ch.override_index + 1}"
                       f"/AUX{ch.override_index - 3} is up)")
            print("       put it DOWN and re-run — this test has to observe the "
                  "off->on transition, and starting engaged also means the "
                  "companion has control the moment you arm")
            return 1
        print("PROPS OFF, DISARMED. Streaming a distinctive pattern.")
        rec.event("stream_start", sticks=[1560, 1440, 1530, 1000],
                  override_mask=ch.override_mask)
        fc.set_stick(roll=1560, pitch=1440, yaw=1530, throttle=1000)
        fc.start_rc_stream()
        time.sleep(0.5)

        print(f"\n>>> flip the OVERRIDE switch (ch{ch.override_index + 1}) UP ...")
        t0 = time.time()
        while not fc.override_active():
            if time.time() - t0 > 60:
                print(BAD, "override never engaged — check the aux mode range "
                           "and msp_override_channels_mask")
                return 1
            time.sleep(0.05)
        rec.event("takeover", wait_s=round(time.time() - t0, 3))
        time.sleep(0.3)
        rc = fc.rc()
        rec.event("takeover_readback", rc=rc, motors=fc.motors())
        print(OK, f"override ACTIVE. FC now reads rc[0:4]={rc[:4]} "
                  "(roll,pitch,yaw,throttle)")
        print(f"       companion sticks are on ch1-4; ARM ch{ch.arm_index + 1} "
              f"still reads {rc[ch.arm_index]} us from the pilot")

        print(f"\n>>> now flip the OVERRIDE switch DOWN ...")
        t0 = time.monotonic()
        while fc.override_active():
            if time.monotonic() - t0 > 60:
                print(BAD, "override never released")
                return 1
            time.sleep(0.01)
        _hb = (time.monotonic() - t0) * 1000
        rec.event("handback", ms=round(_hb, 1), rc=fc.rc())
        print(OK, f"pilot recovered control in {_hb:.0f} ms "
                  "(includes your reaction time)")

        print("\n>>> flip it UP once more; this time the COMPANION stops streaming.")
        t0 = time.time()
        while not fc.override_active():
            if time.time() - t0 > 60:
                print(WARN, "skipped dead-man test")
                return 0
            time.sleep(0.05)
        print("    override active — stopping the stream now (switch stays up)")
        fc.stop_rc_stream("dead-man test")
        t0 = time.monotonic()
        while fc.override_active() and time.monotonic() - t0 < 2.0:
            time.sleep(0.01)
        dt = (time.monotonic() - t0) * 1000
        if fc.override_active():
            print(BAD, "override still active 2 s after the stream stopped — "
                       "the dead-man fallback is NOT working")
            return 1
        rec.event("deadman", ms=round(dt, 1), rc=fc.rc(), motors=fc.motors())
        print(OK, f"stream stopped -> FC dropped the override in {dt:.0f} ms; "
                  "pilot has control")
        print("\nOVERRIDE PATH VERIFIED. Flip the switch down and keep it down.")
        return 0
    finally:
        rec.close()
        fc.close()


# -------------------------------------------------------------- authority

# MSP_RC readback index -> (name, msp_override_channels_mask bit)
# Readback order is roll, pitch, yaw, throttle; mask bits follow TX channel
# order, where ch3 is throttle and ch4 is yaw. The two orders differ — that
# mismatch is the whole reason this check exists.
AUTHORITY_MAP = ((0, "roll", 0), (1, "pitch", 1), (2, "yaw", 3), (3, "throttle", 2))


def t_authority(a):
    """Determine EMPIRICALLY which channels the companion actually drives.

    Streams a distinctive value on each of ch1-4 and reads back what the FC
    used, so the effective msp_override_channels_mask is measured rather than
    trusted. Run this after any mask change: config and firmware disagreeing is
    what makes the wire-envelope check judge the pilot's own stick against the
    companion's limits.

    Disarmed only. Nothing here can spin a motor.
    """
    cfg = load_config(a.config)
    fc = _connect(a)
    try:
        if fc.armed():
            print(BAD, "vehicle is ARMED — disarm before this check")
            return 1
        if not fc.override_active():
            print(BAD, f"override is not active — flip ch{cfg.channels.override_index + 1}"
                       f"/AUX{cfg.channels.override_index - 3} UP and re-run")
            return 1

        pilot = fc.rc()[:4]
        print(f"pilot sticks (r,p,y,thr) : {pilot}")
        test = {"roll": 1560, "pitch": 1440, "yaw": 1530, "throttle": 1080}
        print(f"companion streaming      : {test}")
        fc.set_stick(**test)
        fc.start_rc_stream()
        time.sleep(1.0)
        seen = fc.rc()[:4]
        fc.stop_rc_stream("authority check done")
        print(f"FC actually used         : {seen}")
        print()

        observed = 0
        ambiguous = []
        for idx, name, bit in AUTHORITY_MAP:
            want, was = test[name], pilot[idx]
            got = seen[idx]
            if abs(got - want) <= 3:
                owner, mark = "COMPANION", OK
                observed |= (1 << bit)
            elif abs(got - was) <= 20:
                owner, mark = "pilot", OK
            else:
                owner, mark = "ambiguous", WARN
                ambiguous.append(name)
            print(f"{mark} {name:<9} -> {owner:<10} (companion asked {want}, "
                  f"pilot had {was}, FC used {got})")

        cfgmask = cfg.channels.override_mask
        print()
        print(f"observed mask : {observed}  (0b{observed:04b})")
        print(f"config  mask  : {cfgmask}  (0b{cfgmask:04b})   [channels.override_mask]")
        if ambiguous:
            print(WARN, f"could not classify {ambiguous} — move those pilot sticks "
                        "off the companion's test values and re-run")
            return 1
        if observed == cfgmask:
            print(OK, "config matches the firmware")
            if not (observed & (1 << 2)):
                print("     pilot keeps throttle — companion supplies attitude only")
            return 0
        print(BAD, "MISMATCH — set channels.override_mask to "
                   f"{observed} in {cfg.source}, or change the FC to match.")
        print("     Until they agree, the wire-envelope check policies the wrong "
              "channels: it can judge the pilot's own stick against the "
              "companion's limits and abort the stream mid-test.")
        return 1
    finally:
        fc.close()


# ------------------------------------------------------------------ imu32

def t_imu32(a):
    """Dedicated ESP32/MPU6500 IMU: live state, or calibrate it.

        bringup imu32                # live (q, w, accel) + link health
        bringup imu32 --calibrate    # gyro bias + accel scale, at rest
        bringup imu32 --accel-cal    # per-axis offset/scale, tumble the airframe
        bringup imu32 --accel-cal --poses   # same, from six flat faces instead
        bringup imu32 --axis-map     # sensor->body FRD, needs tilts
    """
    from companion.imu_esp32 import ESP32IMU, GYRO_LSB_PER_DPS
    from companion.math_utils import q_to_R, q_to_euler
    cfg = load_config(a.config)
    c = cfg.imu32
    imu = ESP32IMU(port=c.port or None, cal=c, kp=c.kp, ki=c.ki).start()
    try:
        time.sleep(1.0)
        if imu.frames == 0:
            print(BAD, "no frames from the ESP32 bridge — is esp32/main.py "
                       "installed and running? (mpremote ... cp esp32/main.py :main.py)")
            return 1

        if a.calibrate:
            print("Measuring at rest — do not move the airframe (5 s) ...")
            r = imu.calibrate(5.0)
            if r["moved_during_cal"]:
                print(BAD, "the airframe moved during calibration — re-run still")
                return 1
            print(f"  samples {r['samples']}, frames {r['frames']}, "
                  f"accel spread {r['accel_spread']} counts")
            print(f"  gyro bias   {r['gyro_bias']} counts = {r['gyro_bias_dps']} dps")
            print(f"  accel_per_g {r['accel_per_g']} counts")
            print("\n" + "=" * 62)
            print("paste into config/vehicle.yaml under imu32:\n")
            print("  gyro_bias: [%s]" % ", ".join(str(x) for x in r["gyro_bias"]))
            print("  accel_per_g: %s" % r["accel_per_g"])
            print("=" * 62)
            return 0

        if a.accel_cal:
            # Tumbling is the default: the IMU is bolted into an airframe that
            # does not rest squarely on six faces, and the fit does not care.
            if a.poses:
                return _imu32_accel_cal(imu, cfg)
            return _imu32_tumble_cal(imu, cfg, seconds=a.duration or 60.0)

        if a.axis_map:
            # Against the FC by default: it is a second sensor on the same
            # airframe, so it cannot agree with a wrong answer by accident.
            if a.no_fc:
                return _imu32_axis_map(imu, cfg)
            return _imu32_axis_map_fc(imu, cfg, seconds=a.duration or 60.0)

        print(f"ESP32 IMU — 5 s of live state ({imu.port})")
        t0 = time.time()
        while time.time() - t0 < 5:
            q, w, acc = imu.get_state()
            R = q_to_R(q)
            err = np.degrees(np.arccos(np.clip(
                np.dot(-R[2, :], acc / (np.linalg.norm(acc) + 1e-9)), -1, 1)))
            print(f"  rpy={np.round(q_to_euler(q), 2)} |w|={np.linalg.norm(w):.4f} "
                  f"|a|={np.linalg.norm(acc):.3f} tilt-err={err:.2f}deg")
            time.sleep(0.5)
        print(f"link: {imu.stats()}")
        if not c.verified:
            print(WARN, "axis_map is UNVERIFIED — run: bringup imu32 --axis-map")
        return 0
    finally:
        imu.stop()


def _imu32_hold(imu, seconds=1.5):
    """Mean raw accel over `seconds`, plus the spread that says it was still."""
    samples = []
    stop = time.time() + seconds
    while time.time() < stop:
        a = imu._raw_accel.copy()
        if a.any():
            samples.append(a)
        time.sleep(0.004)
    if len(samples) < 10:
        raise RuntimeError("too few samples — is the bridge streaming?")
    arr = np.array(samples)
    return arr.mean(axis=0), float(arr.std(axis=0).max())


def _fit_accel_ellipsoid(samples):
    """Per-axis offset and scale from accel samples in arbitrary orientations.

    At rest the accelerometer measures exactly 1 g whichever way it points, so
    every sample must land on a sphere of radius 1 g centred on the origin. A
    real part instead produces an ellipsoid, offset from the origin — the
    offset is the zero-g error and the radii are the per-axis scales. Fitting
    that ellipsoid recovers both without ever needing an axis to be vertical,
    which is the point: the IMU is bolted into an airframe that does not sit
    squarely on six faces.

    Solves ``A.a^2 + B.a = 1`` (six unknowns, absorbing the constant term),
    then unpacks: ``o_i = -B_i / 2A_i``, ``D = 1 / (1 + sum o_i^2 A_i)``, and
    ``scale_i = 1 / sqrt(A_i * D)``.
    """
    a = np.asarray(samples, float)
    M = np.hstack([a ** 2, a])
    p, *_ = np.linalg.lstsq(M, np.ones(len(a)), rcond=None)
    A, B = p[:3], p[3:]
    if np.any(A <= 0):
        raise ValueError("degenerate fit — the poses did not cover enough "
                         "orientations (one axis never changed sign)")
    offset = -B / (2.0 * A)
    D = 1.0 / (1.0 + float(np.sum(offset ** 2 * A)))
    scale = 1.0 / np.sqrt(A * D)
    # An ellipsoid can be fitted through any patch of its surface, and the
    # answer off that patch is then extrapolation. Refuse rather than return a
    # confident wrong centre: each axis has to have actually swung through a
    # good fraction of +-1 g for its offset to be observed at all.
    span = (a.max(axis=0) - a.min(axis=0)) / 2.0
    thin = np.where(span < 0.6 * scale)[0]
    if len(thin):
        raise ValueError(
            "sensor axes %s only swung %s of 1 g — turn the airframe further "
            "so every axis points both up and down"
            % (list(thin), ", ".join(f"{span[i] / scale[i]:.0%}" for i in thin)))
    resid = np.linalg.norm((a - offset) / scale, axis=1) - 1.0
    return offset, scale, float(np.std(resid))


def _turned_span(samples):
    """Per-axis half-range of the samples: how far the airframe actually turned.

    Must be seeded from the data, never from zeros — an origin-seeded max/min
    measures distance from the origin instead, so a motionless board sitting
    with gravity at -1960 counts on one axis reports 0.48 g of "coverage" and
    a calibration that never moved looks half done.
    """
    a = np.asarray(samples, float)
    return (a.max(axis=0) - a.min(axis=0)) / 2.0


def _imu32_tumble_cal(imu, cfg, seconds=60.0):
    """Accel calibration by tumbling the airframe — no flat faces needed.

    Samples are kept only while the gyro says the airframe is momentarily
    still, because the fit assumes gravity is the ONLY specific force. Moving
    it slowly and pausing often is what makes this work; waving it about adds
    real acceleration and biases the answer.
    """
    from companion.imu_esp32 import GYRO_LSB_PER_DPS
    STILL_DPS = 8.0
    still_counts = STILL_DPS * GYRO_LSB_PER_DPS
    bias = np.asarray(imu.gyro_bias, float)

    print("PROPS OFF. Pick the airframe up and rotate it slowly through as")
    print("many different orientations as you can — nose up, nose down, on")
    print("each side, upside down. PAUSE for a second in each one.")
    print(f"Collecting for {seconds:.0f}s. It does not need to be tidy.\n")
    _prompt("Ready?")
    # Lead-in, because the first run of this collected 8613 samples of an
    # airframe that had not been picked up yet: the operator needs a moment
    # between "go" and the clock starting, especially when it is launched for
    # them rather than typed by them.
    for n in range(10, 0, -1):
        print(f"    starting in {n} ...")
        time.sleep(1.0)
    print("    GO — turn it now.\n")

    # Seeded from the first sample, NOT from zeros: seeding max/min at 0 makes
    # the span measure distance from the origin instead of how far the
    # airframe actually turned, so a motionless board reported "0.48 g of
    # coverage" purely because gravity sat at -1960 counts on that axis.
    kept = []
    stop = time.time() + seconds
    last = 0.0
    while time.time() < stop:
        a = imu._raw_accel.copy()
        g = imu._raw_gyro.copy() - bias
        if a.any() and np.max(np.abs(g)) < still_counts:
            kept.append(a)
        if time.time() - last > 3.0 and kept:
            last = time.time()
            print(f"    {int(stop - time.time()):3d}s left  still samples "
                  f"{len(kept):5d}  turned so far "
                  + " ".join(f"{v / 2048.0:.2f}g" for v in _turned_span(kept)))
        time.sleep(0.01)

    if not kept:
        print(BAD, "no samples at all — is the bridge streaming?")
        return 1
    span = _turned_span(kept)
    print()
    if len(kept) < 300:
        print(BAD, f"only {len(kept)} still samples — hold each pose longer")
        return 1
    thin = [i for i in range(3) if span[i] < 0.7 * 2048]
    if thin:
        print(BAD, "these sensor axes never saw gravity in both directions: "
                   + ", ".join(str(i) for i in thin))
        print("      turn the airframe further — each axis has to point both "
              "up and down at some point.")
        return 1

    try:
        offset, scale, resid = _fit_accel_ellipsoid(kept)
    except ValueError as e:
        print(BAD, str(e))
        return 1

    print("=" * 62)
    print(f"fitted from {len(kept)} still samples, residual {resid * 100:.2f}% of 1 g")
    for ax in range(3):
        print(f"  sensor {ax}: offset {offset[ax]:+8.1f} counts "
              f"({offset[ax] / scale[ax] * 1000:+6.0f} mg), "
              f"scale {scale[ax]:7.1f} counts/g "
              f"({scale[ax] / 2048.0 - 1.0:+.1%} vs nominal)")
    worst = float(np.degrees(np.arctan(np.max(np.abs(offset)) / scale.mean())))
    print(f"  ⇒ largest offset was worth {worst:.1f} deg of false tilt")
    print("\npaste into config/vehicle.yaml under imu32:\n")
    print("  accel_offset: [%s]" % ", ".join(f"{v:.1f}" for v in offset))
    print("  accel_per_g_axis: [%s]" % ", ".join(f"{v:.1f}" for v in scale))
    print("  accel_per_g: %.1f    # fallback only, now unused" % scale.mean())
    print("=" * 62)
    ok = resid < 0.02
    print((OK if ok else WARN)
          + f" fit residual {resid * 100:.2f}%"
          + ("" if ok else " — over 2%, some samples were not at rest; re-run "
                          "moving more slowly"))
    return 0 if ok else 1


def _imu32_accel_cal(imu, cfg):
    """Six-position accelerometer calibration: per-axis offset and scale.

    Needed because this part is not describable by a single counts-per-g. At
    rest it reported 2400 counts with gravity on sensor z and 2001 with
    gravity on sensor x — a 20% disagreement between two stationary poses. A
    scalar scale silently absorbs that into the gravity DIRECTION, which is
    the one quantity the self-level module acts on.

    You do not need to know which sensor axis is which: each pose is
    classified by whichever axis reads closest to 1 g, and the tool tells you
    which of the six are still missing.
    """
    print("PROPS OFF. Six poses: each face of the board pointing down in turn.")
    print("Any order — the tool works out which is which and tracks what is left.\n")
    seen = {}                                    # (axis, sign) -> mean counts
    for attempt in range(12):
        missing = [(ax, sg) for ax in range(3) for sg in (+1, -1)
                   if (ax, sg) not in seen]
        if not missing:
            break
        print(f"  captured {len(seen)}/6 — still need "
              + ", ".join(f"sensor {ax}{'+' if sg > 0 else '-'}" for ax, sg in missing))
        _prompt("Rest the airframe on a NEW face, completely STILL.")
        mean, spread = _imu32_hold(imu)
        mag = float(np.linalg.norm(mean))
        axis = int(np.argmax(np.abs(mean)))
        sign = int(np.sign(mean[axis]))
        dominance = abs(mean[axis]) / (mag + 1e-9)
        print(f"    raw {np.round(mean, 1)}  |a| {mag:.0f}  spread {spread:.1f} counts")
        if spread > 0.02 * mag:
            print(BAD, "moved during the hold — repeating this pose")
            continue
        if dominance < 0.97:
            print(BAD, f"no axis is within 14 deg of vertical (best {axis} at "
                       f"{np.degrees(np.arccos(dominance)):.0f} deg) — repeating")
            continue
        if (axis, sign) in seen:
            print(WARN, f"sensor axis {axis}{'+' if sign > 0 else '-'} already "
                        "captured — use a different face")
            continue
        seen[(axis, sign)] = mean
        print(OK, f"pose recorded as sensor axis {axis}{'+' if sign > 0 else '-'}")

    if len(seen) != 6:
        print(BAD, f"only {len(seen)}/6 poses captured — nothing written")
        return 1

    offset = np.zeros(3)
    scale = np.zeros(3)
    for ax in range(3):
        hi = seen[(ax, +1)][ax]
        lo = seen[(ax, -1)][ax]
        offset[ax] = (hi + lo) / 2.0        # reading at 0 g
        scale[ax] = (hi - lo) / 2.0         # counts per 1 g
    # Cross-axis offsets: with axis `ax` vertical the OTHER two see 0 g, so
    # their readings are a second, independent estimate of their offset. A
    # disagreement means the poses were not square to gravity.
    resid = []
    for (ax, sg), mean in seen.items():
        for other in range(3):
            if other != ax:
                resid.append(abs(mean[other] - offset[other]))
    print("\n" + "=" * 62)
    print("per-axis, from the six poses:")
    for ax in range(3):
        print(f"  sensor {ax}: offset {offset[ax]:+8.1f} counts "
              f"({offset[ax] / scale[ax] * 1000:+6.0f} mg), "
              f"scale {scale[ax]:7.1f} counts/g "
              f"({scale[ax] / 2048.0 - 1.0:+.1%} vs nominal)")
    print(f"  cross-axis residual: max {max(resid):.0f} counts "
          f"({np.degrees(np.arctan(max(resid) / scale.mean())):.1f} deg of pose error)")
    print("\npaste into config/vehicle.yaml under imu32:\n")
    print("  accel_offset: [%s]" % ", ".join(f"{v:.1f}" for v in offset))
    print("  accel_per_g_axis: [%s]" % ", ".join(f"{v:.1f}" for v in scale))
    print("  accel_per_g: %.1f    # fallback only, now unused" % scale.mean())
    print("=" * 62)
    spread_pct = (scale.max() - scale.min()) / scale.mean()
    print((OK if spread_pct < 0.05 else WARN)
          + f" per-axis scale spread {spread_pct:.1%}"
          + ("" if spread_pct < 0.05 else " — large, but now corrected for"))
    return 0


def _gravity_in_body_from_fc(att):
    """Unit gravity direction in airframe FRD, from the FC's own attitude.

    Only roll and pitch are used. Yaw is deliberately ignored: it is
    unobservable from gravity, it drifts on both estimators, and it cancels
    out of this comparison entirely — which is what makes the check robust.
    """
    r, p = math.radians(att["roll"]), math.radians(att["pitch"])
    # Third row of the world->body rotation, i.e. NED down expressed in body.
    return np.array([-math.sin(p),
                     math.sin(r) * math.cos(p),
                     math.cos(r) * math.cos(p)])


def _specific_force_in_body_from_fc(att):
    """What an ACCELEROMETER on this airframe should read, per the FC.

    Not the gravity direction — its opposite. An accelerometer at rest measures
    specific force, which points UP, while ``_gravity_in_body_from_fc`` returns
    NED down. Fitting the sensor's reading against the down vector instead of
    this cost a whole calibration run: every pair was antiparallel to the
    truth, so the fit was solving for the closest proper rotation to -R. That
    is degenerate — -R is improper, and the nearest rotation to it is any of
    infinitely many 180 deg rotations — so it returned a structured-looking,
    arbitrary answer. It got the x/y swap right and the z sign wrong, which is
    exactly the shape of result that gets believed.
    """
    return -_gravity_in_body_from_fc(att)


def _snap_to_signed_permutation(M):
    """Nearest (axis, sign) map to a measured rotation, greedily by magnitude.

    A mounting is always a signed permutation — the board is bolted at some
    multiple of 90 degrees, not at 37. Snapping to that removes the noise and
    yields the map the config actually wants. Returns (rows, quality) where
    quality is the smallest winning |component|: near 1.0 means the axis was
    unambiguous, near 0.7 means two axes were nearly tied and the mounting is
    NOT square to the airframe.
    """
    M = np.asarray(M, float)
    rows, used, quality = [], set(), 1.0
    order = sorted(range(3), key=lambda i: -np.max(np.abs(M[i])))
    out = [None] * 3
    for i in order:
        cand = [j for j in range(3) if j not in used]
        j = cand[int(np.argmax(np.abs(M[i, cand])))]
        used.add(j)
        out[i] = (j, float(np.sign(M[i, j]) or 1.0))
        quality = min(quality, float(abs(M[i, j])))
    rows = out
    return rows, quality


def _imu32_axis_map_fc(imu, cfg, seconds=60.0):
    """Derive the sensor->body axis map by comparing against the FC's attitude.

    The ESP32 and the flight controller are bolted to the same rigid airframe,
    so at every instant they see the SAME gravity vector expressed in two
    frames that differ by a fixed rotation. Collect enough orientations and
    that rotation is recoverable directly (Kabsch), which beats the
    level/nose-up/roll-right procedure on every axis that matters:

      - no reliance on the operator's idea of "nose up" or "about 30 degrees"
      - no reliance on any single pose being square to anything
      - it is a genuinely INDEPENDENT reference. Betaflight's estimate comes
        from a different die, calibrated by different software, so it cannot
        agree with a wrong ESP32 axis map by construction.

    Requires the ESP32 to be rigidly mounted to the airframe. If it can move
    relative to the FC this is meaningless, and the residual will say so.
    """
    from companion.fc_link import FCLink
    print("PROPS OFF. This compares the ESP32 against the FC's own attitude,")
    print("so the ESP32 must be RIGIDLY MOUNTED to the airframe — if it can")
    print("shift relative to the flight controller the result is worthless.\n")
    if len(cfg.imu32.accel_per_g_axis) != 3:
        print(WARN, "accel_offset/accel_per_g_axis not set — run --accel-cal "
                    "first, or the ~0.17 g offset biases every sample here")

    fc = FCLink(cfg).connect()
    try:
        att = fc.attitude()
        print(f"FC attitude reads roll={att['roll']:+.1f} pitch={att['pitch']:+.1f}")
        _prompt("Ready to tumble?")
        for n in range(10, 0, -1):
            print(f"    starting in {n} ...")
            time.sleep(1.0)
        print("    GO — turn it slowly, pausing a second each time.\n")

        pairs, stop, last = [], time.time() + seconds, 0.0
        bias = np.asarray(imu.gyro_bias, float)
        from companion.imu_esp32 import GYRO_LSB_PER_DPS
        still = 8.0 * GYRO_LSB_PER_DPS
        while time.time() < stop:
            g = imu._raw_gyro.copy() - bias
            if np.max(np.abs(g)) < still:
                a_sensor = imu.sensor_accel()       # offset/scale removed
                if np.linalg.norm(a_sensor) > 1:
                    try:
                        v_fc = _specific_force_in_body_from_fc(fc.attitude())
                    except Exception:
                        continue
                    a_hat = a_sensor / np.linalg.norm(a_sensor)
                    pairs.append((a_hat, v_fc))
            if time.time() - last > 3.0 and pairs:
                last = time.time()
                spread = _turned_span([p[1] for p in pairs])
                print(f"    {int(stop - time.time()):3d}s left  paired samples "
                      f"{len(pairs):4d}  FC gravity swept "
                      + " ".join(f"{v:.2f}" for v in spread))
            time.sleep(0.05)
    finally:
        fc.close()

    if len(pairs) < 60:
        print(BAD, f"only {len(pairs)} paired samples — hold each pose longer")
        return 1
    S = np.array([p[0] for p in pairs])      # sensor frame
    B = np.array([p[1] for p in pairs])      # airframe FRD, per the FC
    if np.any(_turned_span(B) < 0.6):
        print(BAD, "the FC never saw gravity swing far enough on every axis — "
                   "turn the airframe through more orientations")
        return 1

    # Kabsch: the rotation carrying sensor-frame gravity onto body-frame gravity.
    U, _, Vt = np.linalg.svd(S.T @ B)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    M = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    resid = np.degrees(np.mean([
        math.acos(np.clip(np.dot(M @ s, b), -1, 1)) for s, b in zip(S, B)]))
    rows, quality = _snap_to_signed_permutation(M)

    print("\n" + "=" * 62)
    print(f"fitted from {len(pairs)} paired samples")
    print("measured rotation (sensor -> airframe FRD):")
    for r in M:
        print("   [" + "  ".join(f"{v:+.3f}" for v in r) + "]")
    print(f"  mean angle between the two sensors' gravity: {resid:.2f} deg")
    print(f"  snap confidence: {quality:.3f}  (1.0 = square mounting)")
    print("\npaste into config/vehicle.yaml under imu32:\n")
    print("  axis_map:")
    for src, sign in rows:
        print(f"    - [{src}, {sign:+.0f}]")
    print("  verified: true")
    print("=" * 62)
    ok = resid < 8.0 and quality > 0.80
    if not ok:
        print(BAD, f"residual {resid:.1f} deg / confidence {quality:.2f} — the "
                   "two sensors do not agree well enough to trust this.")
        print("      Either the ESP32 is not rigidly mounted, the accel "
              "calibration has not been done, or the FC's own accel needs "
              "calibrating in Betaflight Configurator.")
    else:
        print(OK, "the ESP32 and the FC agree on which way is down")
    return 0 if ok else 1


def _imu32_axis_map(imu, cfg):
    """Determine the sensor->body FRD axis map from level + two tilts.

    Run ``--accel-cal`` first: the axis and sign decisions below come from
    which raw component is largest, and this part's per-axis offsets reach
    ~0.17 g, enough to mis-call a shallow tilt. Offsets are subtracted here if
    they are configured, so the result is only as good as that step.
    """
    print("PROPS OFF. This needs you to physically move the airframe.")
    if not np.any(imu.accel_offset):
        print(WARN, "no accel_offset configured — run: bringup imu32 --accel-cal")
    _prompt("Put the airframe LEVEL and completely STILL.")
    base = imu._raw_accel.copy() - imu.accel_offset
    mag = float(np.linalg.norm(base))
    grav = int(np.argmax(np.abs(base)))
    print(f"    raw accel {np.round(base, 1)}  |a| {mag:.0f}")
    print(f"    gravity on sensor axis {grav}, sign {np.sign(base[grav]):+.0f}")
    zmap = (grav, -float(np.sign(base[grav])))   # level must read -1 g on FRD z

    _prompt("Tilt the NOSE UP about 30 degrees and hold still.")
    up = imu._raw_accel.copy()
    d = up - base
    cand = [i for i in range(3) if i != grav]
    x_axis = cand[int(np.argmax(np.abs(d[cand])))]
    xmap = (x_axis, float(np.sign(d[x_axis])))   # nose up => FRD x positive
    print(f"    raw accel {np.round(up, 1)}")

    _prompt("Roll the airframe RIGHT (right side down) about 30 degrees, hold still.")
    ri = imu._raw_accel.copy()
    d2 = ri - base
    y_axis = [i for i in range(3) if i not in (grav, x_axis)][0]
    ymap = (y_axis, -float(np.sign(d2[y_axis])))  # roll right => FRD y negative
    print(f"    raw accel {np.round(ri, 1)}")

    print("\n" + "=" * 62)
    print("paste into config/vehicle.yaml under imu32:\n")
    print("  # accel scale comes from --accel-cal, not from here: |a| read %.0f"
          % mag)
    print("  # in this pose alone and this part varies 20%% between poses.")
    print("  axis_map:")
    for src, sign in (xmap, ymap, zmap):
        print(f"    - [{src}, {sign:+.0f}]")
    print("  verified: true")
    print("=" * 62)
    ok = len({xmap[0], ymap[0], zmap[0]}) == 3
    print((OK if ok else BAD) + " each FRD axis maps to a distinct sensor axis")
    return 0 if ok else 1


# ----------------------------------------------------------------- motors

MOTOR_MOVES = [
    ("armed idle",      3.0, dict(roll=1500, pitch=1500, yaw=1500, throttle=1000)),
    ("throttle ramp",   5.0, None),                      # handled specially
    ("hold at cap",     2.0, dict(roll=1500, pitch=1500, yaw=1500, throttle=1100)),
    ("ROLL RIGHT",      2.5, dict(roll=1600, pitch=1500, yaw=1500, throttle=1060)),
    ("ROLL LEFT",       2.5, dict(roll=1400, pitch=1500, yaw=1500, throttle=1060)),
    ("PITCH FORWARD",   2.5, dict(roll=1500, pitch=1600, yaw=1500, throttle=1060)),
    ("PITCH BACK",      2.5, dict(roll=1500, pitch=1400, yaw=1500, throttle=1060)),
    ("YAW RIGHT",       2.5, dict(roll=1500, pitch=1500, yaw=1600, throttle=1060)),
    ("YAW LEFT",        2.5, dict(roll=1500, pitch=1500, yaw=1400, throttle=1060)),
    ("back to idle",    2.0, dict(**IDLE_STICKS)),
]


def t_motors(a):
    """PROPS OFF motor-response test. The pilot arms; this drives ch1-4.

    Waits for the pilot to arm, then ramps throttle and steps each attitude
    axis so the motors can be seen responding differentially. Everything is
    recorded. Aborts and stops streaming — which returns control to the pilot —
    on disarm, override release, a motor above the cap, or any link error.
    """
    cfg = load_config(a.config)
    fc = _connect(a)
    ch = cfg.channels
    rec = BenchRecorder("motors", fc, run_dir=cfg.log_dir,
                        state_hz=5.0, deep_hz=2.0).start_sampling()
    try:
        if not fc.rx_link_up():
            print(BAD, "pilot RC link is down")
            return 1
        if not fc.override_active():
            print(BAD, f"override not active — flip ch{ch.override_index+1} UP")
            return 1
        pilot_throttle = not (ch.override_mask & 0b100)
        if pilot_throttle:
            # Mask 11: ch3 belongs to the pilot, by firmware. This is now the
            # standing configuration, so the harness runs attitude-only rather
            # than refusing — the pilot supplies and holds the throttle, which
            # doubles as the cutout. Throttle is still sent every tick and
            # still clamped; the FC simply ignores it.
            print(WARN, "mask %d — the PILOT owns throttle. The companion will "
                        "command attitude only." % ch.override_mask)
            print("       You must hold a small throttle for the motors to turn,"
                  " and chopping it stops them regardless of anything here.")
        boxes = fc.active_boxes()
        if not fc.is_acro():
            lvl = ", ".join(sorted({"ANGLE", "HORIZON"} & boxes))
            print(BAD, f"{lvl} is ACTIVE — this is a levelling mode, not ACRO")
            print(f"       ANGLE is bound to ch{ch.angle_index+1} and is active "
                  f"when that switch is LOW (900-1300); it currently reads "
                  f"{fc.rc()[ch.angle_index]} us")
            print("       Move that switch OUT of the low band for ACRO, then re-run.")
            print("       (ACRO also avoids the levelling I-term winding up on a "
                  "restrained airframe.)")
            return 1
        print(OK, f"ACRO confirmed — active modes: {sorted(boxes) or 'none'}")
        print("      sticks command RATES, so a held stick spins motors "
              "differentially and continuously rather than settling")
        print(f"PROPS OFF. limits: throttle<={fc.limits.thr_cap}, "
              f"deflect+/-{fc.limits.max_deflect}, motor abort {fc.limits.motor_abort}")
        print(f"idle motors (disarmed): {fc.motors()}")

        fc.set_stick(**IDLE_STICKS)
        fc.start_rc_stream()
        rec.event("stream_start", limits=str(fc.limits))

        print(f"\n>>> ARM now with ch{ch.arm_index+1} (throttle is at minimum). "
              f"waiting up to {a.seconds:.0f}s ...")
        t0 = time.monotonic()
        while not fc.armed():
            if fc.abort_reason:
                print(BAD, f"aborted while waiting: {fc.abort_reason}")
                return 1
            if time.monotonic() - t0 > a.seconds:
                print(WARN, "never armed — nothing was commanded. Safe exit.")
                return 0
            time.sleep(0.1)
        rec.event("armed", wait_s=round(time.monotonic() - t0, 2), motors=fc.motors())
        print(OK, f"ARMED. idle motors {fc.motors()}")
        print("    (flip the override switch DOWN at any moment to take control back)")

        moves = MOTOR_MOVES
        if pilot_throttle:
            # Drop the ramp (the pilot supplies throttle) and neutralise ours in
            # every remaining phase, so what this sends can never be confused
            # with what actually flew the motors.
            moves = [(n, t, (dict(k, throttle=1000) if k else None))
                     for n, t, k in MOTOR_MOVES if n != "throttle ramp"]
        for name, dur, sticks in moves:
            if fc.abort_reason:
                break
            print(f"  > {name}")
            rec.event("move", name=name)
            t1 = time.monotonic()
            while (el := time.monotonic() - t1) < dur:
                if name == "throttle ramp":
                    thr = 1000 + (fc.limits.thr_cap - 1000) * (el / dur)
                    fc.set_stick(roll=1500, pitch=1500, yaw=1500, throttle=thr)
                else:
                    fc.set_stick(**sticks)
                if fc.abort_reason:
                    break
                time.sleep(0.05)
            mo = fc.motors()
            rec.event("move_end", name=name, motors=mo, rc=fc.rc()[:4])
            print(f"      motors {mo}   rc {fc.rc()[:4]}")

        if fc.abort_reason:
            print(BAD, f"ABORTED: {fc.abort_reason}")
        else:
            print(OK, "sequence complete")
        return 0
    finally:
        try:
            fc.set_stick(**IDLE_STICKS)
            time.sleep(0.3)
            fc.stop_rc_stream("motor test done")
            time.sleep(0.4)
            print(f"\nstream stopped. motors {fc.motors()}  armed {fc.armed()}")
            print("DISARM (ch%d) and put the override switch DOWN." % (ch.arm_index + 1))
        except Exception:
            pass
        rec.close(abort=fc.abort_reason)
        fc.close()


# ------------------------------------------------------------------- loop

class _NullFC:
    abort_reason = None

    def set_stick(self, **k):
        pass

    def close(self):
        pass


def t_loop(a):
    from companion.command_module import CommandModule
    cfg = load_config(a.config)

    fc = None if a.no_fc else _connect(a)
    if fc:
        fc.start_telemetry()
    elif not a.fake_imu and a.imu == "fc":
        print(BAD, "--no-fc with --imu fc has no attitude source. Use "
                   "--imu esp32 (independent of the FC) or --fake-imu.")
        return 1

    if a.fake_imu:
        imu = FakeIMU()
    else:
        imu = make_imu(a.imu, fc=fc, cfg=cfg)
        if a.imu == "esp32" and not cfg.imu32.verified:
            print(WARN, "ESP32 imu32.axis_map is UNVERIFIED — sensor->body FRD "
                        "is still the identity placeholder. A wrong sign makes "
                        "a self-levelling module drive INTO the tilt. "
                        "Run: bringup imu32 --axis-map")
    imu.start()
    print(f"   attitude source: {a.imu}")
    attitude = (lambda: imu.get_state()[0])
    va = VisionAdapter(cfg.camera, get_attitude_fn=attitude, fake=a.fake_vision)

    # Pinned-module check. The digest is change-detection, not provenance:
    # a re-delivered or swapped .so must not fly unnoticed.
    mod_cfg = getattr(cfg, "module", None)
    if mod_cfg and mod_cfg.verify_on_load and mod_cfg.sha256:
        import hashlib
        so = os.path.join(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))), mod_cfg.so_path)
        if os.path.exists(so):
            h = hashlib.sha256()
            with open(so, "rb") as f:
                for chunk in iter(lambda: f.read(1 << 20), b""):
                    h.update(chunk)
            got = h.hexdigest()
            if got != mod_cfg.sha256:
                print(BAD, "command module digest MISMATCH — refusing to run")
                print(f"       on disk  : {got}")
                print(f"       pinned   : {mod_cfg.sha256}")
                print("       re-run tools/verify_module_delivery.py and "
                      "re-pin only if this change is intended")
                return 1
            print(OK, f"command module pinned digest verified ({got[:16]}...)")
        else:
            print(WARN, f"pinned module not found at {mod_cfg.so_path}")

    lg = FlightLogger(run_dir=cfg.log_dir)
    if a.guidance:
        from companion.guidance_proxy import GuidanceProxy
        g = GuidanceProxy(fc if fc else _NullFC(), imu, va, logger=lg,
                          cfg=cfg.guidance)
        g.start()
        print(f"   guidance host up, binary pinned to {g.sha[:16]}")
    else:
        g = CommandModule(fc if fc else _NullFC(), imu, va, logger=lg)

    hz = cfg.fc.rc_hz
    print(f"!! PROPS OFF. Running the loop (idle command) {a.seconds}s @{hz:.0f} Hz")
    print(f"   log -> {lg.path}")
    lg.event("start", config=cfg.as_dict(), module=getattr(g, "name", type(g).__name__))
    if fc:
        fc.start_rc_stream()

    # engage() starts the module's phase clock; t0 must be captured at the SAME
    # moment, because step() is handed seconds-since-engage. Capturing t0 any
    # earlier (process start, loop construction) hands a phased guidance module
    # a late t on its first tick and mis-sequences it.
    if hasattr(g, "engage"):
        g.engage()
    t0 = time.monotonic()

    period = 1.0 / hz
    next_tick = time.perf_counter()
    try:
        while time.monotonic() - t0 < a.seconds:
            g.step(time.monotonic() - t0)
            if fc and fc.abort_reason:
                lg.event("abort", reason=fc.abort_reason)
                print(BAD, f"aborted: {fc.abort_reason}")
                break
            next_tick += period
            slp = next_tick - time.perf_counter()
            if slp > 0:
                time.sleep(slp)
            else:
                next_tick = time.perf_counter()
    except KeyboardInterrupt:
        lg.event("interrupt")
    finally:
        # stats() is NOT part of the module contract — the reference
        # placeholder happens to provide it, the delivered binary does not.
        # The harness must only rely on what was actually agreed.
        stats = g.stats() if hasattr(g, "stats") else {"ticks": getattr(g, "ticks", None)}
        lg.event("end", **{k: v for k, v in stats.items() if v is not None})
        lg.close()
        imu.stop()
        if hasattr(g, "close"):
            g.close()
        if fc:
            fc.close()
    print(f"loop done: {stats}")
    print(f"analyse:  python3 tools/log_analyze.py {lg.path}")


# -------------------------------------------------------------------- main

def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("cmd", choices=["preflight", "link", "calib-imu", "imu",
                                   "vision", "rc", "authority", "override", "imu32",
                                   "motors", "loop"])
    p.add_argument("--config", default=None, help="path to vehicle.yaml")
    p.add_argument("--port", default=None, help="override the configured FC port")
    p.add_argument("--fake", action="store_true")
    p.add_argument("--fake-imu", action="store_true")
    p.add_argument("--fake-vision", action="store_true")
    p.add_argument("--no-fc", action="store_true")
    p.add_argument("--tracker-url", default=None,
                   help="tracker HTTP telemetry endpoint for the vision test")
    p.add_argument("--seconds", type=float, default=6.0)
    p.add_argument("--imu", choices=["esp32", "fc", "fake"], default="esp32",
                   help="attitude source. esp32 = dedicated MPU6500 at 200 Hz "
                        "(the flight article, costs the FC nothing); fc = FC "
                        "attitude over MSP (bring-up only, 2 txn/tick)")
    p.add_argument("--calibrate", action="store_true",
                   help="imu32: measure gyro bias + accel scale at rest")
    p.add_argument("--accel-cal", action="store_true",
                   help="imu32: per-axis accel offset/scale by tumbling")
    p.add_argument("--poses", action="store_true",
                   help="imu32: with --accel-cal, use six flat faces instead")
    p.add_argument("--duration", type=float, default=0.0,
                   help="imu32: seconds to collect during --accel-cal")
    p.add_argument("--axis-map", action="store_true",
                   help="imu32: determine sensor->body FRD map from tilts")
    p.add_argument("--guidance", action="store_true",
                   help="run the out-of-process guidance host instead of the "
                        "idle placeholder (needs guidance.cert_sha256 set)")
    p.add_argument("-v", "--verbose", action="store_true")
    a = p.parse_args()

    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO,
                        format="%(levelname)s %(name)s: %(message)s")
    handlers = {"preflight": t_preflight, "link": t_link, "calib-imu": t_calib_imu,
                "imu": t_imu, "vision": t_vision, "rc": t_rc,
                "authority": t_authority, "override": t_override,
                "imu32": t_imu32, "motors": t_motors, "loop": t_loop}
    try:
        return handlers[a.cmd](a) or 0
    except MSPError as e:
        print(BAD, e)
        return 1


if __name__ == "__main__":
    sys.exit(main())
