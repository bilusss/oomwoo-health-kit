"""Cycles in which the aggregator's 20 Hz stack stream (published every tick) has a receipt gap > 0.2 s: python3 stack_gaps.py files..."""
import json, sys
for p in sys.argv[1:]:
    rows = [json.loads(l) for l in open(p)]
    n = g = 0
    for r in rows:
        ts = [t for t, k, d in r["ev"] if k == "stack"]
        n += 1
        g += any(b - a > 0.2 for a, b in zip(ts, ts[1:]))
    print(f"{p.split('/')[-1]:22s} cycles={n:3d} cycles_with_aggregator_stream_gap>0.2s={g:3d}")
