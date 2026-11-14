#!/usr/bin/env python3
"""Goal and health-event accounting over soak sessions (M6c): python3 summarize.py results/fullstack/*/sessions.jsonl

Goal categories (from workload records): NavigateToPose goals = succeeded | cancelled mid-drive (test cancel after >= 4 sim s) |
cancelled early (test cancel after 1 sim s: planning / first motion) | unplannable (pose UNREACH, aborted in ~0.01 s, no motion) |
cut by session end | aborted reachable | other; direct recovery probes (spin, backup, wait actions) are counted separately.
Health events: soft stops by class and phase, non-emitting aggregator episodes by phase and whether they caused a soft stop,
stand-in cumulative soft_stops counter delta and the aggregator's healthy_for_sec at the end of each session.
"""

import collections
import json
import sys


def category(g):
    note = g.get("note", "") or ""
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


def main():
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
