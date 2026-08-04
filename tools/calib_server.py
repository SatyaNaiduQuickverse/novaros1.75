#!/usr/bin/env python3
"""Interactive calibration UI, served from the Pi.

    python3 -m tools.calib_server            # then open http://<pi>:8720/

Why a local server and not a hosted page: this has to hold the serial ports to
the ESP32 and the flight controller open. A sandboxed page cannot, so the page
is served from the machine the hardware is plugged into.

What it is for. The CLI calibrations work, but they ask the operator to fly
blind — turn the airframe for ninety seconds and find out at the end whether
the coverage was good enough. Two runs were wasted that way on an airframe
nobody had picked up. This shows, live:

  - which of the six directions have been captured and which are still missing
  - whether the airframe is currently still enough for a sample to count
  - the collected directions plotted, so gaps are visible rather than inferred

The maths is not duplicated here — the fits are imported from tools.bringup so
there is exactly one implementation of each, and the tests cover it.

Read-only with respect to flight: it never streams RC, never arms, and holds
the FC connection only to read attitude.
"""

from __future__ import annotations

import json
import math
import os
import shutil
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

from companion.config import load, DEFAULT_CONFIG
from companion.imu_esp32 import ESP32IMU, GYRO_LSB_PER_DPS

from tools.bringup import (
    _fit_accel_ellipsoid, _gravity_in_body_from_fc, _snap_to_signed_permutation,
    _specific_force_in_body_from_fc, _turned_span,
)


def _perm_matrix(rows):
    """The snapped axis_map as a matrix, for measuring what snapping discards."""
    M = np.zeros((3, 3))
    for i, (src, sign) in enumerate(rows):
        M[i, src] = sign
    return M


def _jsonable(o):
    """numpy scalars are not JSON serialisable, and the failure is silent.

    A stray numpy bool in one result field made the WHOLE state response fail,
    so a completed 60 s calibration was stranded in memory with the page
    showing nothing. Casting at the boundary means one leaked type can never
    take down the response again.
    """
    if isinstance(o, (np.bool_,)):
        return bool(o)
    if isinstance(o, np.integer):
        return int(o)
    if isinstance(o, np.floating):
        return float(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    raise TypeError(f"not JSON serialisable: {type(o).__name__}")

HERE = os.path.dirname(os.path.abspath(__file__))
PAGE = os.path.join(HERE, "calib_ui.html")

STILL_DPS = 8.0
# A sample counts toward a target direction if it is within this angle of it.
# 30 deg gives six caps that do not overlap and are reachable by hand without
# any pose being square to anything.
CAP_DEG = 30.0
# Seconds between pressing Start and the clock starting, so the operator can
# actually pick the airframe up.
LEAD_IN_S = 10.0
# Coverage targets, named the way a person holding a drone thinks. These are
# AIRFRAME directions, classified from the flight controller's own attitude —
# the ESP32's own axes cannot be named yet, because working out which sensor
# axis is which IS step 2. Since the two are rigidly bolted together, covering
# all six airframe directions covers all six sensor directions, so this is
# equivalent for the fit and far easier to act on.
#
# Gravity in body FRD: level puts it on +z, nose up tilts it to -x, rolling
# right tilts it to +y.
BODY_TARGETS = [
    ("Level",      (0, 0, 1),  "upright, sitting as it flies"),
    ("Inverted",   (0, 0, -1), "upside down"),
    ("Nose up",    (-1, 0, 0), "pitch up, nose toward the sky"),
    ("Nose down",  (1, 0, 0),  "pitch down, nose toward the ground"),
    ("Roll right", (0, 1, 0),  "right side down"),
    ("Roll left",  (0, -1, 0), "left side down"),
]
# Fallback when no FC is connected: the sensor's own axes, unnameable.
SENSOR_TARGETS = [("Sensor +X", (1, 0, 0), ""), ("Sensor -X", (-1, 0, 0), ""),
                  ("Sensor +Y", (0, 1, 0), ""), ("Sensor -Y", (0, -1, 0), ""),
                  ("Sensor +Z", (0, 0, 1), ""), ("Sensor -Z", (0, 0, -1), "")]


class SpyFC:
    """Stands in for the flight controller while the module is probed.

    The command module is handed THIS, never the real link, so step 3 can watch
    what it would command without a single byte reaching the FC. It also
    refuses to arm, exactly as the real FCLink does.
    """

    def __init__(self):
        self.last = {}
        self.calls = 0
        self.arm_attempts = 0

    def set_stick(self, roll=None, pitch=None, yaw=None, throttle=None):
        self.last = {k: v for k, v in (("roll", roll), ("pitch", pitch),
                                       ("yaw", yaw), ("throttle", throttle))
                     if v is not None}
        self.calls += 1

    def arm(self, on=True):
        self.arm_attempts += 1


class SelfLevelProbe:
    """Runs the real compiled module against the live IMU, commanding nothing.

    This is the check that cannot be automated away: whether the correction
    OPPOSES the tilt. A sign error here does not misbehave subtly — it drives
    the airframe further into the tilt, and every static check upstream passes
    regardless. So it is watched live, disarmed, with the airframe in the
    operator's hands and the module's output going to a spy.
    """

    def __init__(self, cfg, imu):
        import hashlib
        import importlib.util
        path = cfg.module.so_path
        if not os.path.isabs(path):
            path = os.path.join(os.path.dirname(HERE), path)
        if not os.path.exists(path):
            raise RuntimeError(f"no command module at {path}")
        if cfg.module.verify_on_load and cfg.module.sha256:
            h = hashlib.sha256()
            with open(path, "rb") as f:
                for chunk in iter(lambda: f.read(1 << 20), b""):
                    h.update(chunk)
            if h.hexdigest().lower() != cfg.module.sha256.lower():
                raise RuntimeError("command module digest does not match the "
                                   "pinned value — refusing to load it")
        name = os.path.basename(path).split(".")[0]
        spec = importlib.util.spec_from_file_location(name, path)
        mod = importlib.util.module_from_spec(spec)
        sys.modules[name] = mod
        spec.loader.exec_module(mod)
        self.spy = SpyFC()
        self.mod = mod.CommandModule(self.spy, imu, None, None)
        if hasattr(self.mod, "engage"):
            self.mod.engage()
        self.t0 = time.monotonic()
        self.digest_ok = True

    def step(self):
        return self.mod.step(time.monotonic() - self.t0)


class Session:
    """Owns the hardware and whatever collection run is in progress."""

    def __init__(self):
        self.cfg = load()
        self.imu = None
        self.fc = None
        self.lock = threading.Lock()
        self.mode = "idle"          # idle | accel | axis
        self.samples = []           # sensor-frame unit vectors
        self.body = []              # matching airframe-frame gravity, from the FC
        self.raw = []               # raw counts, for the ellipsoid fit
        self.pairs = []             # (sensor unit, FC body unit) for the axis map
        self.result = None
        self.error = None
        self.started = 0.0
        self.armed_at = 0.0
        self.duration = 0.0
        self._stop = threading.Event()
        # Cached FC attitude. Polled on its own slow thread rather than read on
        # demand: every read costs an MSP transaction from a budget of ~62/s on
        # this board, and sampling it per accel sample would saturate the link
        # and silently slow every other rate.
        self.fc_att = None
        self.fc_grav = None
        self.fc_spec = None
        self.fc_at = 0.0
        self.probe = None           # SelfLevelProbe, when step 3 is watching
        self.probe_error = None
        self.track = []             # (esp roll, esp pitch, fc roll, fc pitch)

    # ------------------------------------------------------------- hardware

    def start_imu(self):
        if self.imu is None:
            self.imu = ESP32IMU(port=self.cfg.imu32.port or None,
                                cal=self.cfg.imu32,
                                kp=self.cfg.imu32.kp, ki=self.cfg.imu32.ki).start()
        return self.imu

    def start_fc(self):
        if self.fc is None:
            from companion.fc_link import FCLink
            self.fc = FCLink(self.cfg).connect()
            threading.Thread(target=self._fc_poll, name="fc-att",
                             daemon=True).start()
        return self.fc

    def _fc_poll(self):
        """20 Hz attitude, still well inside the ~62 txn/s budget. Never raises.

        Was 10 Hz, which put up to 100 ms of staleness into every comparison —
        enough to fabricate 15+ deg of disagreement during a brisk tilt.
        """
        while not self._stop.is_set():
            try:
                att = self.fc.attitude()
                grav = _gravity_in_body_from_fc(att)          # down, for pose naming
                spec = _specific_force_in_body_from_fc(att)   # up, what accels read
                with self.lock:
                    self.fc_att, self.fc_grav, self.fc_spec = att, grav, spec
                    self.fc_at = time.monotonic()
            except Exception:
                with self.lock:
                    self.fc_att = self.fc_grav = self.fc_spec = None
            time.sleep(0.05)

    def _fc_gravity(self):
        """Cached airframe DOWN direction, or None if stale/absent."""
        with self.lock:
            if self.fc_grav is None or time.monotonic() - self.fc_at > 1.0:
                return None
            return self.fc_grav.copy()

    def _fc_specific_force(self):
        """Cached airframe UP direction — what an accelerometer at rest reads."""
        with self.lock:
            if self.fc_spec is None or time.monotonic() - self.fc_at > 1.0:
                return None
            return self.fc_spec.copy()

    def close(self):
        self._stop.set()
        for h in (self.imu, self.fc):
            try:
                if h:
                    h.close() if hasattr(h, "close") else h.stop()
            except Exception:
                pass

    # ------------------------------------------------------------ live view

    def snapshot(self) -> dict:
        imu = self.imu
        if imu is None:
            return {"ok": False, "reason": "IMU not started"}
        raw_a = imu._raw_accel.copy()
        raw_g = imu._raw_gyro.copy() - np.asarray(imu.gyro_bias, float)
        q, w, a = imu.get_state()
        n = float(np.linalg.norm(raw_a))
        rate_dps = float(np.max(np.abs(raw_g)) / GYRO_LSB_PER_DPS)
        from companion.math_utils import q_to_euler
        rpy = [round(float(v), 1) for v in q_to_euler(q)]

        pose = self._current_pose()
        with self.lock:
            covered = self._covered()
            pts = [[round(float(v), 3) for v in s] for s in self.samples[-1500:]]
            nsamp, mode = len(self.samples), self.mode
            now = time.monotonic()
            if mode == "idle":
                phase, left = "idle", 0.0
            elif now < self.armed_at:
                phase, left = "arming", max(0.0, self.armed_at - now)
            else:
                phase = "recording"
                left = max(0.0, self.started + self.duration - now)
            result, error = self.result, self.error

        out = {
            "ok": True,
            "mode": mode,
            "phase": phase,
            "seconds_left": round(left, 1),
            "samples": nsamp,
            "still": rate_dps < STILL_DPS,
            "rate_dps": round(rate_dps, 1),
            "raw_accel": [round(float(v), 1) for v in raw_a],
            "raw_mag": round(n, 1),
            "g_reported": round(float(np.linalg.norm(a)) / 9.80665, 3),
            "rpy": rpy,
            "covered": covered,
            "pose": pose,
            "frame": "airframe" if self._targets() is BODY_TARGETS else "sensor",
            "points": pts,
            "link": imu.stats(),
            "result": result,
            "error": error,
            "calibrated": len(self.cfg.imu32.accel_per_g_axis) == 3,
            "axis_verified": bool(self.cfg.imu32.verified),
            "config_path": self.cfg.source,
        }
        att = self.fc_att
        if att is not None:
            # The two independent estimates of the same physical airframe.
            out["agree"] = {"roll": round(rpy[0] - att["roll"], 2),
                            "pitch": round(rpy[1] - att["pitch"], 2)}
        if self.probe is not None:
            try:
                cmd = self.probe.step()
                tilt = math.hypot(rpy[0], rpy[1])
                # Which way a correction must go is MEASURED from the pilot's
                # transmitter, not assumed. Right-side-down (positive roll) is
                # corrected by rolling LEFT; nose-UP (positive pitch) by
                # pitching the nose DOWN. Those are opposite senses, so a rule
                # that treated the axes symmetrically judged pitch backwards.
                ch = self.cfg.channels
                sign_roll = -1 if ch.roll_right_us > 1500 else 1
                sign_pitch = 1 if ch.nose_down_us > 1500 else -1

                def judge(angle, chan, sign):
                    if abs(angle) <= 8:
                        return None
                    return "opposes" if (cmd.get(chan, 1500) - 1500) * angle * sign > 0 \
                        else "DRIVES INTO THE TILT"
                v_roll = judge(rpy[0], "roll", sign_roll)
                v_pitch = judge(rpy[1], "pitch", sign_pitch)
                verdict = next((v for v in (v_roll, v_pitch)
                                if v == "DRIVES INTO THE TILT"), None) \
                    or next((v for v in (v_roll, v_pitch) if v), None)
                out["selflevel"] = {
                    "roll": cmd.get("roll"), "pitch": cmd.get("pitch"),
                    "yaw": cmd.get("yaw"), "throttle": cmd.get("throttle"),
                    "tilt_deg": round(tilt, 1),
                    "roll_deg": round(float(rpy[0]), 1),
                    "pitch_deg": round(float(rpy[1]), 1),
                    "verdict": verdict,
                    "verdict_roll": v_roll,
                    "verdict_pitch": v_pitch,
                    "sent_to_fc": self.probe.spy.calls,
                    "arm_attempts": self.probe.spy.arm_attempts,
                }
            except Exception as e:
                out["selflevel"] = {"error": str(e)}
        elif self.probe_error:
            out["selflevel"] = {"error": self.probe_error}
        with self.lock:
            att = self.fc_att
        if att is not None:
            out["fc"] = {"roll": round(att["roll"], 1),
                         "pitch": round(att["pitch"], 1)}
        elif self.fc is not None:
            out["fc"] = {"error": "no attitude from the FC"}
        return out

    def _targets(self):
        # Keyed on whether the FC is TALKING, not on whether samples have been
        # collected yet — otherwise the tiles are labelled by sensor axis
        # during the countdown and rename themselves the moment recording
        # starts, which reads as a glitch.
        return BODY_TARGETS if self.fc_grav is not None else SENSOR_TARGETS

    def _covered(self) -> list:
        """How many samples landed in each named direction's cap.

        Counts against the AIRFRAME directions whenever the FC is supplying
        attitude, so the operator is told "nose up" rather than "sensor -X".
        Falls back to the sensor's own axes when there is no FC, where nothing
        can be named until step 2 has run.
        """
        targets = self._targets()
        pool = self.body if targets is BODY_TARGETS else self.samples
        out = [{"name": n, "hint": h, "count": 0} for n, _, h in targets]
        if not pool:
            return out
        S = np.array(pool)
        thresh = math.cos(math.radians(CAP_DEG))
        for row, (_, t, _) in zip(out, targets):
            row["count"] = int(np.count_nonzero(S @ np.array(t, float) > thresh))
        return out

    def _current_pose(self):
        """Which named direction the airframe is in right now, or None."""
        v = self._fc_gravity()
        if v is None:
            imu = self.imu
            a = imu._raw_accel.copy() if imu else None
            if a is None or not a.any():
                return None
            a = imu.sensor_accel()
            v, targets = a / np.linalg.norm(a), SENSOR_TARGETS
        else:
            targets = BODY_TARGETS
        thresh = math.cos(math.radians(CAP_DEG))
        for name, t, _ in targets:
            if float(np.dot(v, np.array(t, float))) > thresh:
                return name
        return None

    # ------------------------------------------------------------ collection

    def begin(self, mode: str, seconds: float, lead_in: float = LEAD_IN_S):
        """Arm, wait out the lead-in, then collect.

        The lead-in is not decoration. Without it the clock starts the instant
        the button is pressed, while the airframe is still on the bench and the
        operator is still reaching for it — which is exactly what happened, and
        looked from the outside like the button doing nothing at all.
        """
        with self.lock:
            if self.mode != "idle":
                raise RuntimeError("a collection is already running")
            self.mode, self.samples, self.raw = mode, [], []
            self.body, self.pairs, self.track = [], [], []
            self.result, self.error = None, None
            self.armed_at = time.monotonic() + lead_in
            self.started, self.duration = self.armed_at, seconds
        threading.Thread(target=self._collect, name="calib", daemon=True).start()

    def cancel(self):
        with self.lock:
            self.mode = "idle"

    def _collect_track(self):
        """Step 3: does the ESP32 follow the real airframe, not just sit still?

        Recorded while MOVING, deliberately. Agreeing at rest only proves the
        static calibration; a wrong sign somewhere downstream still tracks
        perfectly at zero tilt and diverges the moment the airframe moves.
        """
        end = self.started + self.duration
        while time.monotonic() < end:
            with self.lock:
                if self.mode == "idle":
                    return
            with self.lock:
                att, at = self.fc_att, self.fc_at
            q, w, _ = self.imu.get_state()
            from companion.math_utils import q_to_euler
            rpy = q_to_euler(q)
            if att is not None:
                # The gyro is the honest rate. Differencing the FC's attitude
                # is not: it is cached, so repeats of one value difference to
                # zero and a fast swing looks stationary — which is exactly how
                # the worst samples were slipping through as "slow".
                dps = float(np.degrees(np.linalg.norm(w)))
                age = time.monotonic() - at
                with self.lock:
                    self.track.append((time.monotonic(), float(rpy[0]),
                                       float(rpy[1]), att["roll"], att["pitch"],
                                       dps, age))
            time.sleep(0.05)
        self._finish()

    def _collect(self):
        if self.mode == "track":
            return self._collect_track()
        imu = self.imu
        bias = np.asarray(imu.gyro_bias, float)
        still_counts = STILL_DPS * GYRO_LSB_PER_DPS
        while time.monotonic() < self.armed_at:
            with self.lock:
                if self.mode == "idle":
                    return                      # cancelled during the lead-in
            time.sleep(0.05)
        end = self.started + self.duration
        while time.monotonic() < end:
            with self.lock:
                if self.mode == "idle":
                    return                      # cancelled
                mode = self.mode
            g = imu._raw_gyro.copy() - bias
            a = imu._raw_accel.copy()
            if a.any() and np.max(np.abs(g)) < still_counts:
                # Directions come from the CALIBRATED vector: the 316-count
                # zero offset on this part is 8.8 deg of systematic tilt, which
                # would be baked into every axis-map pair. The ellipsoid fit
                # below still gets the raw counts, since finding that offset is
                # what it does.
                cal_a = imu.sensor_accel()
                unit = cal_a / np.linalg.norm(cal_a)
                # Recorded in BOTH modes: step 1 does not need the FC for its
                # maths, but it does need the operator to know which way is
                # "nose up", and only the FC can say that before step 2.
                grav = self._fc_gravity()
                spec = self._fc_specific_force()
                with self.lock:
                    self.samples.append(unit)
                    self.raw.append(a)
                    if grav is not None:
                        self.body.append(grav)
                        if mode == "axis" and spec is not None:
                            # Pair the sensor's reading against what an accel
                            # SHOULD read (up), never against gravity (down).
                            self.pairs.append((unit, spec))
            time.sleep(0.01 if mode == "accel" else 0.05)
        self._finish()

    def _finish(self):
        with self.lock:
            mode, raw, pairs = self.mode, list(self.raw), list(self.pairs)
            self.mode = "idle"
        try:
            if mode == "track":
                self.result = self._fit_track(list(self.track))
            elif mode == "accel":
                self.result = self._fit_accel(raw)
            else:
                self.result = self._fit_axis(pairs)
        except Exception as e:
            self.error = str(e)

    # Above this rate the comparison measures sampling lag, not calibration.
    # The FC's attitude is cached at 10 Hz, so it can be 100 ms stale; at
    # 150 deg/s that alone manufactures 15 deg of apparent disagreement with
    # nothing wrong. Measured: a sweep with mean agreement of 2.3 deg peaked at
    # 15.8 deg purely on the fast segments.
    TRACK_SLOW_DPS = 20.0
    TRACK_MAX_AGE_S = 0.12

    def _fit_track(self, rows):
        if len(rows) < 60:
            raise RuntimeError(f"only {len(rows)} samples — is the FC connected?")
        a = np.array(rows, float)
        esp, fc, dps, age = a[:, 1:3], a[:, 3:5], a[:, 5], a[:, 6]
        d = esp - fc
        moved = float(max(fc[:, 0].max() - fc[:, 0].min(),
                          fc[:, 1].max() - fc[:, 1].min()))
        if moved < 25:
            raise RuntimeError(f"the airframe only moved {moved:.0f} deg — tilt "
                               "it properly, agreeing at rest proves nothing")
        # Judge only where the comparison is meaningful: the airframe nearly
        # still AND the reference freshly read. Either alone is not enough —
        # a stale reference against a moved airframe reads as disagreement.
        slow = (dps < self.TRACK_SLOW_DPS) & (age < self.TRACK_MAX_AGE_S)
        if slow.sum() < 40:
            raise RuntimeError("almost every sample was taken mid-swing — move "
                               "more slowly, or pause briefly at each angle")
        ds = d[slow]
        # Coverage still has to come from the WHOLE sweep: agreeing slowly at
        # one angle is worth nothing, which is what `moved` guards.
        return {
            "kind": "track", "samples": len(rows), "slow_samples": int(slow.sum()),
            "moved_deg": round(moved, 1),
            "mean_roll": round(float(np.mean(np.abs(ds[:, 0]))), 2),
            "mean_pitch": round(float(np.mean(np.abs(ds[:, 1]))), 2),
            "worst_roll": round(float(np.max(np.abs(ds[:, 0]))), 2),
            "worst_pitch": round(float(np.max(np.abs(ds[:, 1]))), 2),
            "worst_any_rate": round(float(np.max(np.abs(d))), 2),
            "peak_rate_dps": round(float(dps.max()), 0),
            "worst_ref_age_ms": round(float(age.max() * 1000), 0),
            "good": bool(np.max(np.abs(ds)) < 8),
        }

    def _fit_accel(self, raw):
        if len(raw) < 300:
            raise RuntimeError(f"only {len(raw)} still samples — hold each "
                               "position for about a second")
        offset, scale, resid = _fit_accel_ellipsoid(raw)
        worst = math.degrees(math.atan(float(np.max(np.abs(offset))) /
                                       float(scale.mean())))
        return {
            "kind": "accel",
            "samples": len(raw),
            "residual_pct": round(resid * 100, 2),
            "false_tilt_deg": round(worst, 1),
            "accel_offset": [round(float(v), 1) for v in offset],
            "accel_per_g_axis": [round(float(v), 1) for v in scale],
            "accel_per_g": round(float(scale.mean()), 1),
            "good": bool(resid < 0.02),
        }

    def _fit_axis(self, pairs):
        if len(pairs) < 60:
            raise RuntimeError(f"only {len(pairs)} paired samples — is the FC "
                               "connected?")
        S = np.array([p[0] for p in pairs])
        B = np.array([p[1] for p in pairs])
        if np.any(_turned_span(B) < 0.6):
            raise RuntimeError("the FC never saw gravity swing far enough on "
                               "every axis — turn it through more orientations")
        U, _, Vt = np.linalg.svd(S.T @ B)
        d = np.sign(np.linalg.det(Vt.T @ U.T))
        M = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
        resid = float(np.degrees(np.mean([
            math.acos(np.clip(np.dot(M @ s, b), -1, 1)) for s, b in zip(S, B)])))
        rows, quality = _snap_to_signed_permutation(M)
        return {
            "kind": "axis",
            "samples": len(pairs),
            "residual_deg": round(resid, 2),
            "confidence": round(float(quality), 3),
            "axis_map": [[int(src), float(sign)] for src, sign in rows],
            "matrix": [[round(float(v), 3) for v in r] for r in M],
            # The exact rotation, kept to full precision. axis_map rounds the
            # mounting to whole 90 deg steps; this preserves the few degrees of
            # real skew that rounding would otherwise leave as standing error.
            "mount_matrix": [[round(float(v), 6) for v in r] for r in M],
            "skew_deg": round(float(np.degrees(np.arccos(np.clip(
                (np.trace(M @ _perm_matrix(rows).T) - 1) / 2, -1, 1)))), 2),
            "good": bool(resid < 8.0 and quality > 0.80),
        }


def apply_to_yaml(path: str, values: dict) -> str:
    """Update keys inside the imu32: block, preserving every comment.

    Deliberately not yaml.safe_dump: config/vehicle.yaml carries the
    measurement and date behind every value, and a round-trip through PyYAML
    would delete all of it. That commentary is the audit trail, so this edits
    lines in place and inserts anything missing at the end of the block.
    """
    shutil.copy2(path, path + ".bak")
    with open(path) as f:
        lines = f.readlines()

    def fmt(v):
        if isinstance(v, (list, tuple)):
            if v and isinstance(v[0], (list, tuple)):
                if all(len(r) == 2 for r in v):          # axis_map pairs
                    return "[" + ", ".join("[%d, %+.1f]" % (a, b) for a, b in v) + "]"
                return "[" + ", ".join(                  # a 3x3 rotation
                    "[" + ", ".join(f"{x:+.6f}" for x in r) + "]" for r in v) + "]"
            return "[" + ", ".join(f"{x}" for x in v) + "]"
        return f"{v}"

    start = next((i for i, ln in enumerate(lines) if ln.rstrip() == "imu32:"), None)
    if start is None:
        raise RuntimeError("no imu32: block in " + path)
    end = len(lines)
    for i in range(start + 1, len(lines)):
        if lines[i].strip() and not lines[i].startswith((" ", "\t")):
            end = i
            break

    remaining, out, i = dict(values), [], start + 1
    head = lines[:start + 1]
    while i < end:
        stripped = lines[i].lstrip()
        hit = next((k for k in remaining if stripped.startswith(k + ":")), None)
        if hit is None:
            out.append(lines[i])
            i += 1
            continue
        out.append(f"  {hit}: {fmt(remaining.pop(hit))}\n")
        i += 1
        # Consume any block sequence that belonged to the old value. axis_map
        # is written as "- [0, +1.0]" over three lines; replacing only the
        # "axis_map:" line would leave those orphaned and the file would no
        # longer parse. Only '-' items are eaten, so comments and sibling keys
        # are safe.
        while i < end and lines[i].lstrip().startswith("- "):
            i += 1
    lines = head + out + lines[end:]
    end = len(head) + len(out)
    if remaining:
        insert = end
        while insert > start and not lines[insert - 1].strip():
            insert -= 1
        block = ["  # written by tools/calib_server.py\n"]
        block += [f"  {k}: {fmt(v)}\n" for k, v in remaining.items()]
        lines[insert:insert] = block

    with open(path, "w") as f:
        f.writelines(lines)
    return path + ".bak"


class Handler(BaseHTTPRequestHandler):
    session: Session = None

    def log_message(self, *a):
        pass                                    # the page polls; do not spam

    def _send(self, code, body, ctype="application/json"):
        data = body if isinstance(body, bytes) else body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        s = self.session
        if self.path in ("/", "/index.html"):
            with open(PAGE, "rb") as f:
                return self._send(200, f.read(), "text/html; charset=utf-8")
        if self.path == "/api/state":
            try:
                return self._send(200, json.dumps(s.snapshot(), default=_jsonable))
            except Exception as e:
                return self._send(200, json.dumps({"ok": False, "reason": str(e)}))
        return self._send(404, json.dumps({"error": "no such path"}))

    def do_POST(self):
        s = self.session
        n = int(self.headers.get("Content-Length") or 0)
        body = json.loads(self.rfile.read(n) or b"{}")
        try:
            if self.path == "/api/connect":
                s.start_imu()
                if body.get("fc"):
                    s.start_fc()
                return self._send(200, json.dumps({"ok": True}))
            if self.path == "/api/start":
                mode = body.get("mode", "accel")
                if mode == "axis":
                    s.start_fc()
                s.begin(mode, float(body.get("seconds", 90)))
                return self._send(200, json.dumps({"ok": True}))
            if self.path == "/api/selflevel":
                if body.get("on"):
                    s.probe_error = None
                    try:
                        s.probe = SelfLevelProbe(s.cfg, s.start_imu())
                    except Exception as e:
                        s.probe, s.probe_error = None, str(e)
                        return self._send(200, json.dumps({"ok": False, "error": str(e)}))
                else:
                    s.probe, s.probe_error = None, None
                return self._send(200, json.dumps({"ok": True}))
            if self.path == "/api/cancel":
                s.cancel()
                return self._send(200, json.dumps({"ok": True}))
            if self.path == "/api/apply":
                r = s.result or {}
                if r.get("kind") == "accel":
                    vals = {"accel_offset": r["accel_offset"],
                            "accel_per_g_axis": r["accel_per_g_axis"],
                            "accel_per_g": r["accel_per_g"]}
                elif r.get("kind") == "axis":
                    vals = {"axis_map": r["axis_map"],
                            "mount_matrix": r["mount_matrix"],
                            "verified": "true"}
                else:
                    raise RuntimeError("no result to apply")
                backup = apply_to_yaml(s.cfg.source or DEFAULT_CONFIG, vals)
                s.cfg = load()
                # Adopt it immediately. Without this the reader keeps the old
                # numbers, the live readout stays wrong, and step 2 would run
                # on uncorrected data — silently undoing step 1.
                if s.imu is not None:
                    s.imu.apply_cal(s.cfg.imu32)
                return self._send(200, json.dumps({"ok": True, "backup": backup}))
        except Exception as e:
            return self._send(200, json.dumps({"ok": False, "error": str(e)}))
        return self._send(404, json.dumps({"error": "no such path"}))


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8720)
    ap.add_argument("--host", default="0.0.0.0")
    a = ap.parse_args()

    Handler.session = Session()
    srv = ThreadingHTTPServer((a.host, a.port), Handler)
    print(f"calibration UI on http://localhost:{a.port}/  (ctrl-C to stop)")
    print("PROPS OFF. This never streams RC and never arms.")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nstopping")
    finally:
        Handler.session.close()


if __name__ == "__main__":
    main()
