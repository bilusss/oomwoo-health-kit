"""Join sleep_trace.so logs with restart_diag cycles (needs pids_before/pids_after in the jsonl).
python3 sleep_events.py cycles.jsonl LOGDIR   -> per cycle: long (>=100 ms) sleeps in Fast DDS threads, by process and caller."""
import collections, glob, json, re, sys
rows = [json.loads(l) for l in open(sys.argv[1])]
names = {}
for r in rows:
    for d in (r["pids_before"], r["pids_after"]):
        for n, p in d.items():
            names[p] = n
ev = []
for f in glob.glob(sys.argv[2] + "/sleep_*.log"):
    pid = int(re.search(r"sleep_(\d+)", f).group(1))
    lines = open(f).read().split("\n"); i = 0
    while i < len(lines):
        m = re.match(r"(\d+\.\d+) (\d+) (\S+) ([\d.-]+)$", lines[i])
        if not m:
            i += 1; continue
        bt = []; j = i + 1
        if j < len(lines) and lines[j] == "BACKTRACE":
            j += 1
            while j < len(lines) and lines[j] and not re.match(r"\d+\.\d+ \d+ ", lines[j]):
                bt.append(lines[j]); j += 1
        s = " ".join(bt)
        key = next((k for k in ("process_acknack", "matched_reader_add", "unsent_change_added", "assert_liveliness") if k in s), "other")
        ev.append((float(m.group(1)), pid, m.group(3), float(m.group(4)), key)); i = j
ev.sort()
tally = collections.Counter()
for r in rows:
    es = [(round(t - r["t0"], 2), names.get(pid, pid), key) for t, pid, th, ms, key in ev
          if ms >= 100 and r["t0"] - 0.5 <= t <= r["t1"]]
    for _, n, key in es:
        tally[(bool(r["flaps"]), n if n not in ("critical_a", "critical_b") else "restarted-or-other-critical", key)] += 1
    print(f"c{r['i']:3d} {r['victim']} flaps={r['flaps']} sleeps(t_from_kill,proc,caller)={es}")
print(dict(tally))
