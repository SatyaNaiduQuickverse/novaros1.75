#!/usr/bin/env python3
"""Summarise a run log.

    python3 tools/log_analyze.py ~/logs/run_*.jsonl

Reports loop timing, how much of the run had a bearing, the longest coast
without one, and any abort.
"""

from __future__ import annotations

import json
import sys


def load(path):
    rows = []
    with open(path) as f:
        for ln in f:
            ln = ln.strip()
            if not ln:
                continue
            try:
                rows.append(json.loads(ln))
            except ValueError:
                pass
    return rows


def summarize_bench(path, rows):
    """Bench-test record: state samples plus timestamped events."""
    samples = [r for r in rows if r.get("sample")]
    events = [r for r in rows if r.get("event")]
    start = next((r for r in events if r["event"] == "bench_start"), {})
    end = next((r for r in events if r["event"] == "bench_end"), {})
    print(f"=== {path} ===")
    print(f"test: {start.get('cmd', '?')}   duration {rows[-1].get('t', 0):.2f}s   "
          f"{len(samples)} state samples")

    if samples:
        # Authority transitions are the whole point of the override test, so
        # report the moment each one happened rather than just the end state.
        prev = None
        for s in samples:
            cur = (s.get("armed"), s.get("override"))
            if prev is not None and cur != prev:
                was = f"armed={prev[0]} override={prev[1]}"
                now = f"armed={cur[0]} override={cur[1]}"
                print(f"  t={s['t']:7.3f}s  {was}  ->  {now}")
            prev = cur
        armed_any = any(s.get("armed") for s in samples)
        ovr_any = any(s.get("override") for s in samples)
        print(f"  was armed at some point : {armed_any}")
        print(f"  override engaged        : {ovr_any}")
        mot = [max(s["motors"]) for s in samples if s.get("motors")]
        if mot:
            print(f"  motor readback          : min {min(mot)}  max {max(mot)}")
        errs = [s for s in samples if s.get("error")]
        if errs:
            print(f"  sampling errors         : {len(errs)}  first={errs[0]['error']}")

    for e in events:
        name = e["event"]
        if name in ("bench_start", "bench_end"):
            continue
        detail = {k: v for k, v in e.items()
                  if k not in ("t", "n", "event")}
        print(f"  t={e['t']:7.3f}s  {name}: {detail}")

    lat = {e["event"]: e.get("ms") for e in events if e.get("ms") is not None}
    if lat:
        print("  latencies:", ", ".join(f"{k} {v} ms" for k, v in lat.items()))
    if end:
        rest = {k: v for k, v in end.items() if k not in ("t", "n", "event")}
        print(f"  result: {rest}")


def summarize(path):
    rows = load(path)
    if not rows:
        print(f"empty log: {path}")
        return
    if any(r.get("event") == "bench_start" for r in rows):
        return summarize_bench(path, rows)
    ticks = [r for r in rows if "event" not in r]
    dur = rows[-1].get("t", 0.0)
    n = len(ticks)
    print(f"=== {path} ===")
    if dur:
        print(f"records {n}  duration {dur:.2f}s  rate {n / dur:.1f} Hz")
    else:
        print(f"records {n}")

    # loop timing: gaps between consecutive ticks
    ts = [r["t"] for r in ticks if "t" in r]
    if len(ts) > 2:
        gaps = [b - a for a, b in zip(ts, ts[1:])]
        gaps.sort()
        print(f"tick gap: median {gaps[len(gaps) // 2] * 1000:.1f} ms  "
              f"p95 {gaps[int(len(gaps) * 0.95)] * 1000:.1f} ms  "
              f"worst {gaps[-1] * 1000:.1f} ms")

    meas = [r for r in ticks if r.get("bearing") is not None]
    frac = 100.0 * len(meas) / n if n else 0.0
    print(f"measured (had a bearing): {len(meas)}/{n} ({frac:.0f}%) -> "
          f"{'GOOD track' if frac > 60 else 'SPARSE / lost track'}")

    rngs = [r["rng"] for r in ticks if isinstance(r.get("rng"), (int, float))]
    if rngs:
        print(f"range: start {rngs[0]:.1f} m  min {min(rngs):.2f} m  end {rngs[-1]:.1f} m")

    confs = sorted(r["conf"] for r in ticks if isinstance(r.get("conf"), (int, float)))
    if confs:
        print(f"detector confidence: median {confs[len(confs) // 2]:.2f}  min {confs[0]:.2f}")

    maxgap, last_t = 0.0, None
    for r in ticks:
        if r.get("bearing") is not None:
            last_t = r.get("t")
        elif last_t is not None:
            maxgap = max(maxgap, r.get("t", 0.0) - last_t)
    print(f"longest coast (no bearing): {maxgap * 1000:.0f} ms")

    thr = [r["sticks"][3] for r in ticks if isinstance(r.get("sticks"), list)]
    if thr:
        print(f"throttle commanded: min {min(thr)}  max {max(thr)} us")

    aborts = [r for r in rows if r.get("abort")]
    if aborts:
        print(f"ABORT: {aborts[0]['abort']} at t={aborts[0].get('t', 0):.2f}s")

    events = [(r.get("t"), r.get("event")) for r in rows if r.get("event")]
    if events:
        print("events: " + ", ".join(f"{e}@{t:.2f}s" for t, e in events))


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit("usage: python3 tools/log_analyze.py <run.jsonl> [more.jsonl ...]")
    for p in sys.argv[1:]:
        summarize(p)
        print()
