"""Connector-side half of the out-of-process guidance split.

Presents the ordinary ``(fc, imu, vision).step(t)`` seam to the control loop,
while the guidance itself runs in a spawned subprocess that holds no serial
handle and therefore cannot reach the flight controller at all. Clamping
happens *here*, in the process that owns the port, on data crossing the
boundary — which is what makes the clamps load-bearing again rather than
advisory.

    proxy = GuidanceProxy(fc, imu, vision, logger=lg, cfg=cfg.guidance)
    proxy.engage()
    while running:
        proxy.step(time.monotonic() - t0)
    proxy.close()

Wire protocol (connector -> host, host -> connector):

    ("init",   opts|None)                        -> -
    ("engage", v0, p0)                           -> -
    ("tick",   t, q, w, accel, bearing)          -> ("cmd", (roll, pitch, yaw, thr))
    ("stop",)                                    -> -

``t`` is engagement-relative seconds. ``q`` is ``[w,x,y,z]``, ``w`` is rad/s and
``accel`` m/s², both body FRD, all as plain lists so the boundary carries no
numpy version coupling. ``bearing`` is ``None`` or
``(u_world_list, range_m|None, conf, t_capture)``.

The host's reply sticks are UNCLAMPED desired values by design; every clamp is
applied on this side.

Three failure modes all end the same way — the RC stream stops and Betaflight
returns control to the pilot within ~250 ms:

  * host does not reply inside ``poll_timeout_s``  (hung or slow guidance)
  * host process dies                              (EOF on the pipe)
  * host reports a binary whose hash is not the certified one (refused at launch)
"""

from __future__ import annotations

import glob
import hashlib
import logging
import multiprocessing as mp
import os
import sys
import time

from .safety import IDLE_STICKS

log = logging.getLogger("companion.guidance")

ABORT_HOST_TIMEOUT = "guidance host did not reply in time"
ABORT_HOST_DIED = "guidance host process died"
ABORT_HOST_PROTOCOL = "guidance host protocol violation"


SERIAL_PREFIXES = ("/dev/ttyACM", "/dev/ttyUSB", "/dev/ttyAMA", "/dev/ttyS",
                   "/dev/serial/")


def reachable_serial_devices() -> list[str]:
    """Serial devices this process can reach through an open fd.

    Deliberately ignores fds 0-2: a controlling terminal is a character device
    too, and flagging it would make this check useless noise.
    """
    import stat
    found = []
    for link in glob.glob("/proc/self/fd/*"):
        try:
            fd = int(os.path.basename(link))
            if fd < 3:
                continue
            st = os.fstat(fd)
            if not stat.S_ISCHR(st.st_mode):
                continue
            real = os.path.realpath(link)
            if real.startswith(SERIAL_PREFIXES):
                found.append(f"fd {fd} -> {real} rdev={hex(st.st_rdev)}")
        except (OSError, ValueError):
            pass
    return found


def _child_entry(conn, host_module: str, so_dir: str, cert_sha256: str | None):
    """Entry point inside the SPAWNED child.

    Two things happen here before any guidance code is imported, both of which
    exist because the isolation can be defeated in ways that are invisible from
    the connector side:

    1. The child proves it cannot reach a serial port, and exits if it can.
       Two known ways it could: being forked instead of spawned (fd inheritance),
       or spawn re-importing an unguarded ``__main__`` that opens the port at
       module scope. Both are caught here, before the guidance loads, and both
       fail closed — the parent sees EOF and aborts at launch.

    2. The host module is imported *here*, never in the connector process. If
       the connector imported it merely to obtain this function reference, a
       host that loads its .so at import time would pull the guidance binary
       straight back into the trusted process and undo the split.
    """
    leaked = reachable_serial_devices()
    if leaked:
        print("[guidance-host] ISOLATION FAILURE, refusing to start — this "
              "process can reach: " + "; ".join(leaked), file=sys.stderr,
              flush=True)
        print("[guidance-host] check the start method is 'spawn' and that the "
              "connector's entry point is under `if __name__ == \"__main__\"`",
              file=sys.stderr, flush=True)
        os._exit(90)

    import importlib
    mod = importlib.import_module(host_module)
    mod.run(conn, so_dir, cert_sha256)


def so_digest(so_dir: str, so_path: str | None = None) -> tuple[str, list[str]]:
    """SHA-256 of the guidance binary, computed on the connector side.

    A single .so hashes to exactly what ``sha256sum`` prints for that file, so
    it can be compared against a digest emitted by the build. Several .so files
    produce a composite over (name, bytes) pairs which will NOT match any
    single-file digest — pass ``so_path`` explicitly in that case.
    """
    files = [so_path] if so_path else sorted(glob.glob(os.path.join(so_dir, "*.so")))
    if not files:
        raise FileNotFoundError(f"no .so found in {so_dir!r}")
    h = hashlib.sha256()
    if len(files) == 1:
        with open(files[0], "rb") as fh:
            for chunk in iter(lambda: fh.read(1 << 20), b""):
                h.update(chunk)
    else:
        for f in files:
            h.update(os.path.basename(f).encode() + b"\0")
            with open(f, "rb") as fh:
                for chunk in iter(lambda: fh.read(1 << 20), b""):
                    h.update(chunk)
    return h.hexdigest(), files


class GuidanceProxy:
    name = "guidance-oop"

    def __init__(self, fc, imu, vision, logger=None, cfg=None):
        if cfg is None:
            from .config import GuidanceConfig
            cfg = GuidanceConfig()
        self.fc = fc
        self.imu = imu
        self.vision = vision
        self.log = logger
        self.cfg = cfg
        self.ticks = 0
        self.engaged = False
        self._proc = None
        self._pipe = None
        self._dead = False
        self._last_t = None
        self._epoch_warned = False
        self.rtt_worst = 0.0
        self.rtt_total = 0.0
        self.sha = None

    # ------------------------------------------------------------- launch

    def start(self) -> "GuidanceProxy":
        """Spawn the host, pin the binary's hash, and refuse anything else."""
        expected = self.cfg.cert_sha256
        if not expected:
            raise ValueError(
                "guidance.cert_sha256 is not set — refusing to launch an "
                "unpinned guidance binary. Take the digest from the build."
            )

        # Hash it ourselves rather than trusting the host's self-report alone:
        # a swapped binary is exactly the case this is meant to catch.
        local, files = so_digest(self.cfg.so_dir, self.cfg.so_path or None)
        if local != expected:
            raise PermissionError(
                f"guidance binary hash mismatch — refusing to launch.\n"
                f"  files    : {files}\n"
                f"  on disk  : {local}\n"
                f"  certified: {expected}"
            )

        # spawn, never fork: a forked child inherits this process's open file
        # descriptors, including the FC's serial port, which would leave the
        # guidance able to write raw MSP despite being 'isolated'. Measured on
        # this Pi — see docs/COMMAND_MODULE_INTERFACE.md.
        ctx = mp.get_context("spawn")
        parent, child = ctx.Pipe()
        self._proc = ctx.Process(
            target=_child_entry,
            args=(child, self.cfg.host_module, self.cfg.so_dir, expected),
            daemon=True, name="guidance-host")
        self._proc.start()
        # Drop our copy of the child end, otherwise the pipe never reports EOF
        # when the host dies and a dead host would look merely slow.
        child.close()
        self._pipe = parent

        if not parent.poll(self.cfg.start_timeout_s):
            self._kill()
            raise TimeoutError(
                f"guidance host sent no ready message within "
                f"{self.cfg.start_timeout_s}s")
        tag, sha = parent.recv()
        if tag != "ready":
            self._kill()
            raise RuntimeError(f"guidance host said {tag!r}, expected 'ready'")
        if sha != expected:
            self._kill()
            raise PermissionError(
                f"guidance host loaded {sha}, certified {expected}")
        self.sha = sha
        log.info("guidance host up, binary pinned to %s", sha[:16])
        self._send(("init", self.cfg.init_opts or None))
        return self

    def _kill(self):
        self._dead = True
        try:
            if self._pipe:
                self._pipe.close()
        except Exception:
            pass
        if self._proc is not None and self._proc.is_alive():
            self._proc.terminate()
            self._proc.join(1.0)
            if self._proc.is_alive():
                self._proc.kill()

    # --------------------------------------------------------------- abort

    def _abort(self, reason: str):
        """Stop the RC stream. Betaflight hands the pilot control in ~250 ms."""
        if not self._dead:
            log.warning("guidance aborted: %s", reason)
            if self.log:
                self.log.event("guidance_abort", reason=reason)
        self._dead = True
        try:
            self.fc.stop_rc_stream(reason)
        except Exception:
            pass

    def _send(self, msg) -> bool:
        if self._dead:
            return False
        if self._proc is not None and not self._proc.is_alive():
            self._abort(ABORT_HOST_DIED)
            return False
        try:
            self._pipe.send(msg)
            return True
        except (BrokenPipeError, OSError, EOFError) as e:
            self._abort(f"{ABORT_HOST_DIED}: {e}")
            return False

    # ----------------------------------------------------------- lifecycle

    def engage(self, v0=None, p0=None) -> None:
        if self._proc is None:
            self.start()
        self.engaged = True
        self.ticks = 0
        self._last_t = None
        # Start from a known-safe command so nothing is inherited from an
        # earlier run: throttle at the floor, sticks centred.
        try:
            self.fc.set_stick(**IDLE_STICKS)
        except Exception:
            pass
        self._send(("engage", list(v0 or [0.0, 0.0, 0.0]), list(p0 or [0.0, 0.0, 0.0])))
        if self.log:
            self.log.event("engage", module=self.name, sha=self.sha)

    def _check_epoch(self, t):
        if self._epoch_warned:
            return
        if self._last_t is None and t > 1.0:
            log.warning("first step() t=%.1fs, expected ~0 — t must be seconds "
                        "since engage()", t)
            self._epoch_warned = True
        elif self._last_t is not None and t < self._last_t:
            log.warning("step() t went backwards (%.3f -> %.3f)", self._last_t, t)
            self._epoch_warned = True

    # -------------------------------------------------------------- tick

    def step(self, t) -> dict:
        self._check_epoch(t)
        self._last_t = t
        q, w, accel = self.imu.get_state()
        b = self.vision.bearing(t)
        bearing = None if b is None else (
            [float(x) for x in b.u_world],
            None if b.range_m is None else float(b.range_m),
            float(b.conf), float(b.t))

        sticks = dict(IDLE_STICKS)
        if self._dead:
            return sticks

        t0 = time.perf_counter()
        if not self._send(("tick", float(t), [float(x) for x in q],
                           [float(x) for x in w], [float(x) for x in accel],
                           bearing)):
            return sticks

        # A hung guidance process must trip the dead-man exactly like a hung
        # in-process loop. Waiting indefinitely would leave the last frame
        # streaming with nothing behind it.
        if not self._pipe.poll(self.cfg.poll_timeout_s):
            self._abort(f"{ABORT_HOST_TIMEOUT} "
                        f"({self.cfg.poll_timeout_s * 1000:.0f} ms)")
            return sticks
        try:
            reply = self._pipe.recv()
        except (EOFError, OSError) as e:
            self._abort(f"{ABORT_HOST_DIED}: {e}")
            return sticks

        rtt = time.perf_counter() - t0
        self.rtt_worst = max(self.rtt_worst, rtt)
        self.rtt_total += rtt

        try:
            tag, cmd = reply
            roll, pitch, yaw, thr = cmd
        except (TypeError, ValueError):
            self._abort(f"{ABORT_HOST_PROTOCOL}: {reply!r}")
            return sticks
        if tag != "cmd":
            self._abort(f"{ABORT_HOST_PROTOCOL}: expected 'cmd', got {tag!r}")
            return sticks

        # THE clamp point. Values arriving here are unclamped desired sticks;
        # set_stick -> aetr_frame applies the envelope, the AETR ordering and
        # the four-channel restriction, in the process that owns the port.
        sticks = {"roll": roll, "pitch": pitch, "yaw": yaw, "throttle": thr}
        self.fc.set_stick(**sticks)
        self.ticks += 1

        if self.log:
            self.log.log(q=q, w=w, acc=accel,
                         bearing=(b.u_world if b else None),
                         conf=(b.conf if b else None),
                         rng=(b.range_m if b else None),
                         want=[roll, pitch, yaw, thr],
                         rtt_ms=round(rtt * 1000, 3),
                         mode=self.name,
                         abort=getattr(self.fc, "abort_reason", None))
        return sticks

    def close(self):
        self._send(("stop",))
        time.sleep(0.02)
        self._kill()

    def stats(self) -> dict:
        return {"ticks": self.ticks,
                "rtt_mean_ms": round(1000 * self.rtt_total / max(self.ticks, 1), 3),
                "rtt_worst_ms": round(1000 * self.rtt_worst, 3),
                "sha": (self.sha or "")[:16]}
