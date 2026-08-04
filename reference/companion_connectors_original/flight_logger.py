import json, time, os


class FlightLogger:
    def __init__(self, path=None, run_dir="~/logs", also_stdout=False):
        run_dir = os.path.expanduser(run_dir)
        os.makedirs(run_dir, exist_ok=True)
        if path is None:
            path = os.path.join(run_dir, "run_%d.jsonl" % int(time.time()))
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
        return v

    def log(self, **fields):
        rec = {"t": round(time.time() - self._t0, 4), "n": self.n}
        for k, v in fields.items():
            rec[k] = self._enc(v)
        line = json.dumps(rec)
        self._f.write(line + "\n")
        if self._stdout:
            print(line, flush=True)
        self.n += 1
        return rec

    def event(self, name, **fields):
        return self.log(event=name, **fields)

    def flush(self):
        self._f.flush()

    def close(self):
        try:
            self._f.close()
        except Exception:
            pass


# wiring (per control tick):
#   lg = FlightLogger()
#   lg.log(q=q, w=w, acc=acc_body, vel=v_est, bearing=(u_world if meas else None),
#          conf=conf, rng=rng_est, vcmd=vcmd, sticks=sticks, mode=mode)
#   ... on state changes:  lg.event("lock"), lg.event("launch"), lg.event("terminal")
#   lg.close()  at end
