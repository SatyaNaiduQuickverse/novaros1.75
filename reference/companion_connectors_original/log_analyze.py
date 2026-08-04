import json, sys, math


def load(path):
    rows = []
    with open(path) as f:
        for ln in f:
            ln = ln.strip()
            if ln:
                try:
                    rows.append(json.loads(ln))
                except ValueError:
                    pass
    return rows


def summarize(path):
    rows = load(path)
    if not rows:
        print("empty log:", path); return
    dur = rows[-1].get("t", 0)
    n = len(rows)
    events = [(r.get("t"), r.get("event")) for r in rows if r.get("event")]
    meas = [r for r in rows if r.get("bearing") is not None]
    rngs = [r["rng"] for r in rows if isinstance(r.get("rng"), (int, float))]
    confs = [r["conf"] for r in rows if isinstance(r.get("conf"), (int, float))]
    print(f"=== {path} ===")
    print(f"records {n}  duration {dur:.2f}s  rate {n/dur:.1f} Hz" if dur else f"records {n}")
    if rngs:
        print(f"range: start {rngs[0]:.1f} m  min {min(rngs):.2f} m  end {rngs[-1]:.1f} m")
        print(f"  -> closest approach = {min(rngs):.2f} m")
    frac = 100.0 * len(meas) / n if n else 0
    print(f"measured (had a bearing): {len(meas)}/{n}  ({frac:.0f}%)  "
          f"-> {'GOOD track' if frac > 60 else 'SPARSE/lost track'}")
    if confs:
        confs.sort()
        print(f"detector confidence: median {confs[len(confs)//2]:.2f}  min {confs[0]:.2f}")
    # longest gap without a measurement (coast)
    gap = maxgap = 0; last_t = None
    for r in rows:
        if r.get("bearing") is not None:
            last_t = r.get("t")
        elif last_t is not None:
            gap = (r.get("t", 0) - last_t)
            maxgap = max(maxgap, gap)
    print(f"longest coast (no bearing): {maxgap*1000:.0f} ms")
    if events:
        print("events: " + ", ".join(f"{e}@{t:.2f}s" for t, e in events))


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("usage: python3 log_analyze.py <run.jsonl> [more.jsonl ...]"); sys.exit(1)
    for p in sys.argv[1:]:
        summarize(p); print()
