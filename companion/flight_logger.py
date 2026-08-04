"""Per-tick JSONL flight logger.

One JSON object per line, one line per control tick, plus named events. Written
with line buffering so a power cut costs at most the current line — this Pi is
power-off-prone during bench work.

    lg = FlightLogger()
    lg.event("start", config=cfg.as_dict())
    lg.log(q=q, w=w, acc=acc, bearing=u_world, conf=conf, rng=rng,
           sticks=[roll, pitch, yaw, throttle], mode="idle")
    lg.event("abort", reason=fc.abort_reason)
    lg.close()
"""

from __future__ import annotations

import json
import os
import time


class FlightLogger:
    def __init__(self, path: str | None = None, run_dir: str = "~/logs",
                 also_stdout: bool = False):
        run_dir = os.path.expanduser(run_dir)
        os.makedirs(run_dir, exist_ok=True)
        if path is None:
            stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
            path = os.path.join(run_dir, f"run_{stamp}.jsonl")
        self.path = os.path.expanduser(path)
        self._f = open(self.path, "a", buffering=1)
        self._t0 = time.time()
        self._stdout = also_stdout
        self.n = 0

    def _enc(self, v):
        try:
            import numpy as np
            if isinstance(v, np.ndarray):
                return [round(float(x), 5) for x in v.ravel()]
            if isinstance(v, (np.floating, np.integer)):
                return round(float(v), 5)
        except Exception:
            pass
        if isinstance(v, float):
            return round(v, 5)
        if isinstance(v, (list, tuple)):
            return [self._enc(x) for x in v]
        if isinstance(v, dict):
            return {k: self._enc(x) for k, x in v.items()}
        return v

    def log(self, **fields) -> dict:
        rec = {"t": round(time.time() - self._t0, 4), "n": self.n}
        for k, v in fields.items():
            rec[k] = self._enc(v)
        line = json.dumps(rec)
        self._f.write(line + "\n")
        if self._stdout:
            print(line, flush=True)
        self.n += 1
        return rec

    def event(self, name: str, **fields) -> dict:
        return self.log(event=name, **fields)

    def flush(self):
        self._f.flush()

    def close(self):
        try:
            self._f.close()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
