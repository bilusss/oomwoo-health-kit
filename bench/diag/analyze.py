"""Summarise restart_diag jsonl files: flaps per cycle, timing of flaps, gaps. Host python, no ROS."""
import json, sys, statistics as st

def load(p):
    return [json.loads(l) for l in open(p)]

def summ(path):
    rows = load(path)
    n = len(rows); fl = [r for r in rows if r["flaps"]]
    nf = sum(len(r["flaps"]) for r in rows)
    sf = sum(1 for r in rows if r.get("stack_flaps_post_rearm"))
    rearm_fail = sum(1 for r in rows if r.get("rearmed") is False)
    ts = sorted(t for r in fl for t in r["flaps"])
    med = st.median(ts) if ts else None
    print(f"{path.split('/')[-1]:34s} cycles={n:4d} cycles_with_flap={len(fl):3d} flaps={nf:3d} "
          f"agg_healthy_to_fault_cycles={sf:3d} rearm_fail={rearm_fail} median_t_after_rearm={med}")

for p in sys.argv[1:]:
    summ(p)
