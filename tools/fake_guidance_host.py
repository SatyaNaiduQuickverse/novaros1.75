"""Reference guidance host — a stand-in for the real one, to exercise the
connector-side boundary handler before the guidance .so exists.

Implements the agreed wire protocol and nothing else. It holds no serial
handle, and being spawned (never forked) it has no access to the flight
controller's file descriptor either — which is the property the whole split
exists to create.

    run(conn, so_dir, cert_sha256)

Fault injection for testing the connector's abort paths, via the
``NOVA_HOST_FAULT`` environment variable:

    hang      stop replying after a few ticks   -> poll timeout abort
    die       exit mid-run                      -> EOF / dead-process abort
    badproto  reply with a malformed message    -> protocol abort
    badsha    report a hash that is not the file's -> refused at launch
    rogue     try to reach the FC directly      -> should find nothing to reach
"""

from __future__ import annotations

import glob
import hashlib
import os
import stat
import sys


def _digest(so_dir: str) -> str:
    files = sorted(glob.glob(os.path.join(so_dir, "*.so")))
    if not files:
        raise FileNotFoundError(f"no .so in {so_dir!r}")
    h = hashlib.sha256()
    with open(files[0], "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _reachable_character_devices() -> list[str]:
    """What device nodes can this process actually reach through open fds?

    The isolation claim is that a spawned guidance host cannot touch the FC.
    This is how that claim gets checked rather than asserted.
    """
    found = []
    for link in glob.glob("/proc/self/fd/*"):
        try:
            st = os.fstat(int(os.path.basename(link)))
            if stat.S_ISCHR(st.st_mode) and st.st_rdev != 0:
                found.append(f"{os.path.realpath(link)} rdev={hex(st.st_rdev)}")
        except (OSError, ValueError):
            pass
    return found


def run(conn, so_dir: str, cert_sha256: str | None = None) -> None:
    fault = os.environ.get("NOVA_HOST_FAULT", "")

    if fault == "rogue":
        devs = _reachable_character_devices()
        print(f"[host] reachable character devices: {devs or 'NONE'}",
              file=sys.stderr, flush=True)

    sha = _digest(so_dir)
    conn.send(("ready", "0" * 64 if fault == "badsha" else sha))

    t_engage = None
    ticks = 0
    while True:
        try:
            msg = conn.recv()
        except EOFError:
            return
        tag = msg[0]

        if tag == "stop":
            return
        if tag == "init":
            continue
        if tag == "engage":
            _, v0, p0 = msg
            t_engage, ticks = 0.0, 0
            continue
        if tag != "tick":
            continue

        _, t, q, w, accel, bearing = msg
        ticks += 1

        if fault == "die" and ticks > 5:
            os._exit(1)
        if fault == "hang" and ticks > 5:
            import time as _t
            _t.sleep(30)
        if fault == "badproto" and ticks > 5:
            conn.send("not-a-tuple")
            continue

        # Stand-in law: hold level, and steer toward the bearing when there is
        # one. Deliberately emits UNCLAMPED desired values, including some out
        # of range, so the connector-side clamp is what constrains them.
        roll = pitch = yaw = 1500.0
        thr = 1000.0
        if fault == "overrange":
            # Every channel far outside the envelope, independent of inputs,
            # so the connector-side clamp is what is being measured.
            conn.send(("cmd", (2400.0, 300.0, -800.0, 2000.0)))
            continue
        if bearing is not None:
            u, rng, conf, t_cap = bearing
            yaw = 1500.0 + 4000.0 * u[1]      # deliberately over-range
            pitch = 1500.0 - 4000.0 * u[2]
            thr = 1000.0 + 900.0 * conf       # deliberately above the bench cap
        conn.send(("cmd", (roll, pitch, yaw, thr)))
