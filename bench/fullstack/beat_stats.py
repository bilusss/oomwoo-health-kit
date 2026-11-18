#!/usr/bin/env python3
"""M9 recon: beat period statistics per component from beat_trace.py files (ROS-free).

  python3 beat_stats.py TRACE.jsonl... [--json]

Also `age_before_refresh`: for each pair of consecutive beats, sim_now at the receipt of the later one minus stamp_sec of the earlier one = the oldest age the
aggregator could have seen on the earlier beat (gap plus delivery delay); and `recommended_max_age_s` (rule fixed BEFORE the M9 recon data was read):
max(3 x median period, 1.5 x the largest age_before_refresh over ALL pooled files), rounded UP to a multiple of 0.05 s.

Per component and segment: n beats, the period between consecutive beats by their stamp_sec (the node's ROS clock = sim seconds), p50 / p95 / p99 /
max, the same by receive time (host monotonic), and the age at receipt (sim_now - stamp_sec: transport and scheduling delay). Segments: all;
moving / stationary (the robot's /odom twist at the later beat of the pair; |v| >= 0.03 m/s or |w| >= 0.05 rad/s = moving); for the dock
additionally the dock_drive state at the later beat. A pair of beats more than JOIN_GAP_S apart that straddles a trace restart is not
special-cased: one trace file = one session; periods are only taken inside a file.
"""

import json
import math
import sys

MOVING_V, MOVING_W = 0.03, 0.05


def pct(xs, q):
    """Nearest-rank percentile (q in 0..100) of a non-empty list."""
    xs = sorted(xs)
    return xs[max(0, min(len(xs) - 1, math.ceil(q / 100.0 * len(xs)) - 1))]


def load(path):
    rows = [json.loads(ln) for ln in open(path) if ln.strip()]
    return rows


def analyse(rows):
    """{component: {segment: {...}}} from the rows of ONE trace file."""
    odom = [(r[1], abs(r[2]) >= MOVING_V or abs(r[3]) >= MOVING_W) for r in rows if r[0] == "odom"]
    states = [(r[1], r[3]) for r in rows if r[0] == "state"]
    beats = {}
    for r in rows:
        if r[0] == "beat":
            beats.setdefault(r[3], []).append(r)
    out = {}
    oi = si = 0
    for comp, bs in beats.items():
        seg = {}
        prev = None
        oi = si = 0
        mov = None
        st = None
        for b in bs:
            t = b[1]
            while oi < len(odom) and odom[oi][0] <= t:
                mov = odom[oi][1]
                oi += 1
            while si < len(states) and states[si][0] <= t:
                st = states[si][1]
                si += 1
            keys = ["all", "moving" if mov else "stationary"] if mov is not None else ["all"]
            if st is not None:
                keys.append("state_" + st)
            if prev is not None:
                for k in keys:
                    d = seg.setdefault(k, {"stamp": [], "recv": [], "age": [], "abr": [], "seq_gaps": 0, "n": 0})
                    d["stamp"].append(b[4] - prev[4])
                    d["abr"].append(b[2] - prev[4])
                    d["recv"].append(t - prev[1])
                    d["n"] += 1
                    if b[5] is not None and prev[5] is not None and b[5] != prev[5] + 1:
                        d["seq_gaps"] += 1
            for k in keys:
                seg.setdefault(k, {"stamp": [], "recv": [], "age": [], "abr": [], "seq_gaps": 0, "n": 0})["age"].append(b[2] - b[4])
            prev = b
        out[comp] = seg
    return out


def summarize(seg):
    def q(xs):
        return None if not xs else {"p50": pct(xs, 50), "p95": pct(xs, 95), "p99": pct(xs, 99), "max": max(xs)}
    return {"periods_n": len(seg["stamp"]), "period_stamp_s": q(seg["stamp"]), "period_recv_s": q(seg["recv"]), "age_at_receipt_s": q(seg["age"]), "age_before_refresh_s": q(seg["abr"]),
            "seq_gaps": seg["seq_gaps"]}


def merge(analyses):
    """Pool several files (periods are concatenated, never taken across files)."""
    tot = {}
    for a in analyses:
        for comp, segs in a.items():
            for k, d in segs.items():
                t = tot.setdefault(comp, {}).setdefault(k, {"stamp": [], "recv": [], "age": [], "abr": [], "seq_gaps": 0, "n": 0})
                for f in ("stamp", "recv", "age", "abr"):
                    t[f] += d[f]
                t["seq_gaps"] += d["seq_gaps"]
                t["n"] += d["n"]
    return tot


def recommended_max_age(pooled_all):
    """Rule written down before the data was read (module docstring); pooled_all = the 'all' segment of one component after merge()."""
    p50 = pct(pooled_all["stamp"], 50)
    worst = max(pooled_all["abr"])
    v = max(3.0 * p50, 1.5 * worst)
    return round(math.ceil(v / 0.05 - 1e-9) * 0.05, 2)


def main(argv):
    as_json = "--json" in argv
    files = [a for a in argv if not a.startswith("--")]
    pooled = merge([analyse(load(p)) for p in files])
    res = {c: {k: summarize(d) for k, d in sorted(segs.items())} for c, segs in sorted(pooled.items())}
    for c, segs in res.items():
        if "all" in segs:
            segs["all"]["recommended_max_age_s"] = recommended_max_age(pooled[c]["all"])
    if as_json:
        print(json.dumps(res, indent=1))
        return
    for c, segs in res.items():
        for k, s in segs.items():
            p, r, a = s["period_stamp_s"], s["period_recv_s"], s["age_at_receipt_s"]
            if not p:
                continue
            print(f"{c:18s} {k:16s} n={s['periods_n']:6d} stamp p50 {p['p50']:.3f} p95 {p['p95']:.3f} p99 {p['p99']:.3f} max {p['max']:.3f} | "
                  f"recv p50 {r['p50']:.3f} p99 {r['p99']:.3f} max {r['max']:.3f} | age p50 {a['p50']:.3f} max {a['max']:.3f} | to-refresh max {s['age_before_refresh_s']['max']:.3f} | seq gaps {s['seq_gaps']}"
                  + (f" | recommended max_age {s['recommended_max_age_s']}" if "recommended_max_age_s" in s else ""))


if __name__ == "__main__":
    main(sys.argv[1:])
