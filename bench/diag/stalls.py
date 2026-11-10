"""For each flap cycle list per-stream delivery stalls (inter-arrival > 90 ms or latency > 40 ms) relative to restart.
python3 stalls.py file.jsonl [--all]   (--all: every cycle, not only flaps)"""
import json, sys
rows = [json.loads(l) for l in open(sys.argv[1])]
allc = "--all" in sys.argv
for r in rows:
    if not (r["flaps"] or allc):
        continue
    tr = r.get("t_restart"); t0 = r["t0"]; off = (tr - t0) if tr else 0
    streams = {}
    for t, k, d in r["ev"]:
        if k == "comp": streams.setdefault(d["id"], []).append((t, (d["wall"] - d["stamp"]) * 1000))
        elif k == "mcu": streams.setdefault("mcu", []).append((t, (d["wall"] - d["stamp"]) * 1000))
        elif k == "stack": streams.setdefault("stack", []).append((t, 0.0))
    out = []
    for s, ev in sorted(streams.items()):
        for (ta, _), (tb, lat) in zip(ev, ev[1:]):
            if (tb - ta) * 1000 > 90 and s != "mcu" and s != "stack_x" or (s in ("mcu",) and (tb - ta) * 1000 > 90) or lat > 40:
                out.append(f"{s}@{tb - off:.2f}(gap{(tb-ta)*1000:.0f},lat{lat:.0f})")
    fr = r.get("first_run_after_kill")
    print(f"c{r['i']:3d} {r['victim']} restart@{off:.2f} run@{fr} flaps={r['flaps']} :: " + " ".join(out))
