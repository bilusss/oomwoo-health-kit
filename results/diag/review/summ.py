"""Summarise stopmode runs: python3 summ.py run.jsonl TRACE_DIR (host python)."""
import collections, glob, json, re, sys

rows = [json.loads(l) for l in open(sys.argv[1])]
ev = []
for f in glob.glob(sys.argv[2] + "/sleep_*.log") if len(sys.argv) > 2 else []:
    pid = int(re.search(r"sleep_(\d+)", f).group(1))
    lines = open(f).read().split("\n")
    for i, l in enumerate(lines):
        m = re.match(r"(\d+\.\d+) (\d+) (\S+) ([\d.-]+)$", l)
        if m and float(m.group(4)) >= 100:
            bt = " ".join(lines[i + 1:i + 16]) if i + 1 < len(lines) and lines[i + 1] == "BACKTRACE" else ""
            key = next((k for k in ("process_acknack", "matched_reader_add", "send_acknack", "DomainParticipantFactoryD1")
                        if k in bt), "other")
            ev.append((float(m.group(1)), pid, m.group(3), float(m.group(4)), key))
names = {}
for r in rows:
    for n, p in r["pids_before"].items():
        names[p] = "silent" if n == "extra" else n
n = len(rows)
flap = [r for r in rows if r["flaps"]]
agg_side = probe_side = 0
for r in rows:
    comp_t = [t for t, k, d in r["ev"] if k == "comp"]
    for a, g in r["stack_gaps"]:
        alive = any(a + 0.02 < t < a + g - 0.02 for t in comp_t)
        if alive:
            agg_side += 1
        else:
            probe_side += 1
print(f"{sys.argv[1].split('/')[-1]}: cycles={n} cycles_with_soft_stop={len(flap)} soft_stops={sum(len(r['flaps']) for r in rows)} "
      f"cycles_with_stack_gap>0.2s={sum(1 for r in rows if r['stack_gaps'])} (gaps with probe alive={agg_side}, probe silent={probe_side}) "
      f"rc={collections.Counter(r['rc'] for r in rows)} max_exit_s={max(r['t_exit'] - r['t_stop'] for r in rows):.3f} "
      f"leftover_shm_cycles={sum(1 for r in rows if r['shm_new_after'])}")
print(" soft stop times from node start:", sorted(t for r in rows for t in r["flaps"]))
print(" stack gaps (start,len):", [g for r in rows for g in r["stack_gaps"]])
tally = collections.Counter()
for t, pid, th, ms, key in sorted(ev):
    r = next((r for r in rows if r["t0"] - 0.5 <= t <= r["t1"]), None)
    who = names.get(pid, pid)
    tally[(who, key, bool(r and r["flaps"]))] += 1
    if r and r["flaps"]:
        print(f"  c{r['i']} sleep {ms:.0f}ms in {who} {th} via {key} at t={t - r['t0']:.3f} flaps={r['flaps']} gaps={r['stack_gaps']}")
print(" 333ms-sleep tally (process, caller, in_flap_cycle):", dict(tally))
