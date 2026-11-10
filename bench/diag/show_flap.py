"""Print the event timeline around the first flap in a cycle: python3 show_flap.py file.jsonl [cycle_index] [window_s]"""
import json, sys
rows = [json.loads(l) for l in open(sys.argv[1])]
fl = [r for r in rows if r["flaps"]]
idx = int(sys.argv[2]) if len(sys.argv) > 2 else 0
r = fl[idx]
win = float(sys.argv[3]) if len(sys.argv) > 3 else 0.6
fr = r["first_run_after_kill"]; t_flap = fr + r["flaps"][0]
print("cycle", r["i"], r["victim"], "first_run", fr, "flap at", t_flap, "all flaps", r["flaps"])
last = {}
for t, k, d in r["ev"]:
    if not (t_flap - win <= t <= t_flap + 0.15):
        continue
    if k == "comp": s = f"comp {d['id']} seq={d['seq']} lat_ms={(d['wall']-d['stamp'])*1000:.1f}"
    elif k == "mcu": s = f"mcu seq={d['seq']}"
    elif k == "stack": s = f"stack {d['s']} bl={d['bl']}"
    elif k == "state": s = f"STATE {d['s']} {d['reason']}"
    else: continue
    print(f"{t:8.4f} {s}")
