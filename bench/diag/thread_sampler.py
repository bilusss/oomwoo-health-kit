"""Poor-man's strace: sample every thread of the kit's processes via /proc (needs root inside ohk-dev; write to /tmp).

python3 thread_sampler.py OUT SECONDS [interval_ms]
Each line: t_mono pid proc tid comm state sysno arg0 (sysno column = kernel wchan; arg0 = utime/stime ticks; syscall file is not readable in this container). Observation only.
"""
import os, sys, time

out = open(sys.argv[1], "w", buffering=1 << 20)
dur = float(sys.argv[2]); iv = float(sys.argv[3]) / 1000 if len(sys.argv) > 3 else 0.002
PAT = {"health_monitor_node": "agg", "mcu_standin_node": "mcu", "roster_publisher": "roster"}
procs = {}  # pid -> name
t_end = time.monotonic() + dur; t_scan = 0

def scan():
    for pid in filter(str.isdigit, os.listdir("/proc")):
        pid = int(pid)
        if pid in procs:
            continue
        try:
            cl = open(f"/proc/{pid}/cmdline", "rb").read().replace(b"\0", b" ").decode(errors="replace")
        except OSError:
            continue
        name = None
        for k, v in PAT.items():
            if k in cl and "thread_sampler" not in cl:
                name = v
        if "fake_component_node" in cl and "thread_sampler" not in cl:
            name = "comp:" + (cl.split("component_id:=")[1].split()[0] if "component_id:=" in cl else "?")
        if name:
            procs[pid] = name

while True:
    now = time.monotonic()
    if now > t_end:
        break
    if now - t_scan > 0.1:
        t_scan = now; scan()
    for pid, name in list(procs.items()):
        try:
            tids = os.listdir(f"/proc/{pid}/task")
        except OSError:
            del procs[pid]; continue
        for tid in tids:
            try:
                base = f"/proc/{pid}/task/{tid}/"
                comm = open(base + "comm").read().strip()
                stat = open(base + "stat").read()
                state = stat[stat.rindex(")") + 2]
                sysno = open(base + "wchan").read().strip() or "-"
                f = stat[stat.rindex(")") + 2:].split()
                arg0 = f"{f[11]}/{f[12]}"  # utime/stime ticks
            except (OSError, IndexError, ValueError):
                continue
            out.write(f"{now:.4f} {pid} {name} {tid} {comm} {state} {sysno} {arg0}\n")
    time.sleep(iv)
out.close()
