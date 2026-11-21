#!/usr/bin/env python3
"""Goal and health-event accounting over soak sessions (M6c): python3 summarize.py results/fullstack/*/sessions.jsonl
(M7: python3 summarize.py --by-task FILE... prints one block per task and profile.)

Goal categories (from workload records): NavigateToPose goals = succeeded | cancelled mid-drive (test cancel after >= 4 sim s) |
cancelled early (test cancel after 1 sim s: planning / first motion) | unplannable (pose UNREACH, aborted in ~0.01 s, no motion) |
cut by session end | aborted reachable | other; direct recovery probes (spin, backup, wait actions) are counted separately.
Health events: soft stops by class and phase, non-emitting aggregator episodes by phase and whether they caused a soft stop,
stand-in cumulative soft_stops counter delta and the aggregator's healthy_for_sec at the end of each session.
"""

import collections
import datetime as dt
import json
import statistics
import sys


def category(g):
    note = g.get("note", "") or ""
    if g["pose"].startswith("DOCK:"):
        return "dock_" + g["result"] if not (g["result"] == "canceled" and note == "session ended") else "cut_by_session_end"
    if g["pose"] == "COVERAGE":
        return "coverage_run_cut_by_session_end" if g["result"] == "canceled" else "coverage_run_" + g["result"]
    if "scenario" in g:   # M7 recovery task: goal with a wall (persistent / transient / none)
        w = (g["scenario"] or {}).get("wall") or "no_wall"
        if g["result"] == "canceled" and note == "session ended":
            return "cut_by_session_end"
        return f"recovery_{w}_{g['result']}"
    if g["pose"] == "RECOVERY":
        return "recovery_probe"
    if g["pose"] == "UNREACH":
        return "unplannable"
    if g["result"] == "succeeded":
        return "succeeded"
    if g["result"] == "canceled" and note == "session ended":
        return "cut_by_session_end"
    if g["result"] == "canceled" and "test cancel after 1.0" in note:
        return "cancelled_early_1s"
    if g["result"] == "canceled" and note.startswith("test cancel"):
        return "cancelled_mid_drive"
    if g["result"] == "aborted":
        return "aborted_reachable"
    return "other_" + g["result"]


def _ts(iso):
    return dt.datetime.fromisoformat(iso).timestamp()


MOTION_V, MOTION_W, MOTION_CMD_AGE_S = 0.02, 0.05, 1.0
HARNESS_WINDOW_S = (0.0, 0.5)   # a soft stop counts as "after a harness action" if it follows the action by 0.0-0.5 s


def active_seconds(d):
    """Seconds of the measured window in which the job moved the robot, from the session record (M7 review).

    Continuous workloads (goal loops, dock cycles, recovery scenarios) are active for the whole window. A coverage job finishes its sweep
    (planner log `complete`) and then idles for the rest of the session, so for coverage tasks the active time ends at the LAST motion
    evidence: an uncommanded-motion event (odometry above the thresholds), or a health event whose context shows motion (odometry above the
    thresholds or a command younger than 1 s). A stop with an old command and a still robot is idle time, not activity. Uses only what the
    session record keeps (every soft stop and episode; the first 60 uncommanded-motion events, which for a Nav2-driven sweep is all of them).
    No evidence at all -> 0.0. Result is at most the window length. It is an upper-bound style estimate: the active period is taken to start at 0.
    """
    sim_s = d.get("sim_s") or 0.0
    if d.get("task") not in COVERAGE_TASKS:
        return sim_s
    t0 = _ts(d["window_start_wall"])
    end = 0.0
    for e in (d.get("uncommanded_motion") or {}).get("events", []):
        end = max(end, _ts(e["wall"]) - t0 + (e.get("duration_s") or 0.0))
    for e in d.get("health_events", []):
        if not idle_event(e):
            end = max(end, _ts(e["wall"]) - t0)
    return min(end, sim_s)


def idle_event(e):
    """A health event whose context shows a still robot and no recent command (odom below the thresholds, command older than 1 s)."""
    ctx = e.get("fault_ctx") or e
    moving = (ctx.get("odom_v") or 0.0) >= MOTION_V or (ctx.get("odom_w") or 0.0) >= MOTION_W
    age = ctx.get("cmd_age_wall_s")
    fresh = age is not None and age < MOTION_CMD_AGE_S
    return not moving and not fresh and "odom_v" in ctx


COVERAGE_TASKS = ("coverage", "coverage_reactive", "coverage_reactive_cmd30",   # every task whose workload is "coverage" (test checks tasks.TASKS)
                  "coverage_inode", "coverage_reactive_inode", "coverage_inode_recon", "coverage_reactive_inode_recon")


def harness_related(soft_wall, action_epochs):
    """True if the soft stop (ISO wall string) followed a harness gz action by 0.0-0.5 s."""
    t = _ts(soft_wall)
    return any(HARNESS_WINDOW_S[0] <= t - a <= HARNESS_WINDOW_S[1] for a in action_epochs)


def task_coverage(d):
    return d.get("task") in COVERAGE_TASKS


def aggregate(paths):
    """{(task, profile): totals} over the sessions in the JSONL files; invalid sessions are only counted."""
    rows = collections.OrderedDict()
    for path in paths:
        for line in open(path):
            d = json.loads(line)
            r = rows.setdefault((d.get("task", "nav_slam"), d.get("profile")), collections.defaultdict(float))
            r["sessions"] += 1
            if not d.get("valid"):
                r["invalid"] += 1
                continue
            r["sim_h"] += (d.get("sim_s") or 0) / 3600.0
            r["active_h"] += active_seconds(d) / 3600.0
            r["dist_m"] += d.get("distance_m") or 0
            r["soft"] += d.get("soft_stops_n") or 0
            r["gaps"] += d.get("hb_gaps_n") or 0
            r["uncmd"] += (d.get("uncommanded_motion") or {}).get("n", 0)
            ls = [x[1] for x in (d.get("loadavg_series") or [])]   # M8: host 1-min load average sampled every 10 s during the session
            if ls:
                r.setdefault("load_med", []).append(statistics.median(ls))
                r["load_max"] = max(r["load_max"], max(ls))
            r.setdefault("blamed", collections.Counter()).update(
                "+".join(sorted(e.get("blamed") or [])) or "none" for e in d.get("health_events", []) if e["kind"] == "soft_stop")
            r.setdefault("phase", collections.Counter()).update(e.get("phase") or "?" for e in d.get("health_events", []) if e["kind"] == "soft_stop")
            acts = [a["epoch"] for g in d.get("goals", []) for a in g.get("harness_actions", [])]
            for e in d.get("health_events", []):
                if e["kind"] == "soft_stop":
                    r["soft_idle" if (task_coverage(d) and idle_event(e)) else "soft_active"] += 1
                    key = "soft_after_harness" if harness_related(e["wall"], acts) else "soft_other"
                    r[key] += 1
            r.setdefault("cat", collections.Counter()).update(category(g) for g in d.get("goals", []))
            r.setdefault("cls", collections.Counter()).update((d.get("health_event_tables") or {}).get("soft_stops_by_class", {}))
            r.setdefault("tally", collections.Counter()).update(
                {f"{a}.{k}": v for a, c in (d.get("action_tally") or {}).items() for k, v in c.items() if k == "started" and a in
                 ("spin", "backup", "wait", "drive_on_heading", "navigate_to_pose")})
    return rows


def by_task(paths):
    """M7 table: one block per (task, profile) over valid sessions: sim hours AND active hours (coverage jobs idle after the sweep ends),
    goals / attempts by category, distance, soft stops by class and per active hour, heartbeat gaps, BT recovery actions started."""
    for (task, prof), r in aggregate(paths).items():
        rate = (r["soft_active"] / r["active_h"]) if r["active_h"] > 0 else float("nan")
        print(f"{task:24s} {str(prof):9s} sessions {int(r['sessions'])} (invalid {int(r['invalid'])})  sim_h {r['sim_h']:.2f}  active_h {r['active_h']:.2f}  "
              f"distance {r['dist_m'] / 1000:.2f} km  soft_stops {int(r['soft'])} {dict(r.get('cls', {}))}, {int(r['soft_active'])} while active = {rate:.1f} per active h, {int(r['soft_idle'])} while idle "
              f"(0-0.5 s after a harness gz action: {int(r.get('soft_after_harness', 0))}, other: {int(r.get('soft_other', 0))})  "
              f"hb_gaps>0.2s {int(r['gaps'])}  uncommanded {int(r['uncmd'])}")
        if r.get("load_med"):
            print(f"   host load (1-min avg of the host, sampled every 10 s): median of session medians {statistics.median(r['load_med']):.1f}, "
                  f"max {r['load_max']:.1f} of 12 logical cores")
        if r.get("blamed"):
            print(f"   soft stops by blamed component(s): {dict(r['blamed'])}; by phase: {dict(r['phase'])}")
        print(f"   goals/attempts: {dict(r.get('cat', {}))}")
        print(f"   actions started: {dict(r.get('tally', {}))}")


def main():
    if len(sys.argv) > 2 and sys.argv[1] == "--by-task":
        return by_task(sys.argv[2:])
    goals, soft_cls, soft_phcls, ep_ph, valid = collections.Counter(), collections.Counter(), collections.Counter(), collections.Counter(), 0
    sessions, delta, hfs, uncmd = 0, [], [], 0
    for path in sys.argv[1:]:
        for line in open(path):
            d = json.loads(line)
            sessions += 1
            valid += bool(d.get("valid"))
            for g in d.get("goals", []):
                goals[category(g)] += 1
            t = d.get("health_event_tables") or {}
            soft_cls.update(t.get("soft_stops_by_class", {}))
            soft_phcls.update(t.get("soft_stops_by_phase_class", {}))
            ep_ph.update(t.get("non_emitting_episodes_by_phase_soft_stop", {}))
            delta.append(d.get("standin_soft_stops_delta"))
            hfs.append((d.get("aggregator_at_end") or {}).get("healthy_for_sec"))
            uncmd += (d.get("uncommanded_motion") or {}).get("n", 0)
    nav = sum(v for k, v in goals.items() if k != "recovery_probe")
    print(f"sessions {sessions} (valid {valid}); NavigateToPose goals {nav}: " + ", ".join(f"{k} {v}" for k, v in sorted(goals.items())))
    print("soft stops by class:", dict(soft_cls), "by phase/class:", dict(soft_phcls))
    print("non-emitting aggregator episodes by phase / soft stop:", dict(ep_ph))
    print("stand-in soft_stops counter delta per session:", delta)
    print("aggregator healthy_for_sec at end:", [None if h is None else round(h, 1) for h in hfs])
    print("motion without a command newer than 0.15 s (events):", uncmd)


if __name__ == "__main__":
    main()
