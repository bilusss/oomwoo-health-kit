#!/usr/bin/env python3
"""List every soft stop of M8 soak sessions with its context and the watcher's own reason from the retained stack log (ROS-free).

  python3 bench/fullstack/m8_stops.py results/fullstack/soak_m8_*.jsonl [--logs results/fullstack/m8]

For each soft stop: session, wall time, phase of the workload, blamed component(s), odometry and command age at the start of the fault episode, and the
`topic_heartbeat` warnings (`<component>: no heartbeat, <reason>`, throttled to one per component and 5 s) within 2 s before the stop in the session's stack log
(`stack_log` field of the session record, found under --logs/<instance>/). A stop whose warning is missing is reported as such.
"""

import argparse
import datetime as dt
import glob
import json
import os
import re

WARN = re.compile(r"\[topic_heartbeat[^\]]*\] \[WARN\] \[(\d+\.\d+)\] \[topic_heartbeat\]: (\S+): no heartbeat, (.*)")


def stops(paths):
    for path in paths:
        for line in open(path):
            d = json.loads(line)
            for e in d.get("health_events", []):
                if e["kind"] == "soft_stop":
                    yield d, e


def warnings_near(log_path, t_epoch, before=2.0, after=0.3):
    out = []
    if not log_path or not os.path.exists(log_path):
        return None
    for ln in open(log_path, errors="replace"):
        m = WARN.search(ln)
        if m and t_epoch - before <= float(m.group(1)) <= t_epoch + after:
            out.append((round(float(m.group(1)) - t_epoch, 2), m.group(2), m.group(3).strip()))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="+")
    ap.add_argument("--logs", default=os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "results", "fullstack", "m8"))
    a = ap.parse_args()
    n = 0
    for d, e in stops(a.files):
        n += 1
        t = dt.datetime.fromisoformat(e["wall"]).timestamp()
        log = None
        if d.get("stack_log"):
            # two instances started in the same second share the file name: the record's own instance directory decides
            hits = glob.glob(os.path.join(a.logs, str(d.get("instance") or "*"), d["stack_log"]))
            log = hits[0] if hits else None
        w = warnings_near(log, t)
        ctx = e.get("fault_ctx") or {}
        print(f"{d['task']} s{d['session']} {e['wall']} phase={e.get('phase')} (goal {e.get('goal')}, {e.get('since_goal_start_s')} s in) class={e.get('class')} "
              f"blamed={e.get('blamed')} odom_v={ctx.get('odom_v')} odom_w={ctx.get('odom_w')} cmd_age={ctx.get('cmd_age_wall_s')} nonzero={ctx.get('last_cmd_nonzero')}")
        if w is None:
            print("    stack log not found")
        elif not w:
            print("    no watcher warning within 2 s before the stop")
        for rel, comp, why in (w or []):
            print(f"    {rel:+.2f} s {comp}: {why}")
    print(f"{n} soft stops")


if __name__ == "__main__":
    main()
