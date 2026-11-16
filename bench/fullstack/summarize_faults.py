#!/usr/bin/env python3
"""Fault-trial table (M7): python3 summarize_faults.py FILE... [--latest-label-per-task | --label=X[,Y]] [--task=T] [--detail]

One row per (task, fault, profile) over VALID trials: n, exposed, detected (stand-in soft stop at any time in the observation), detected within the
hold (soft stop <= hold_s after the signal, 5 s), first zero command at the gate (min / median / max s from the signal), standstill, distance to
standstill (m), rotation after the signal (degrees, from the ground-truth yaw rate), robot still moving at the end, blamed components, resumed.

Exposure (defined AFTER the trials were seen, M7 review): a trial is exposed if the robot was actually in the motion the fault is about when the
signal arrived. For `recovery` faults (behavior_server) that means a spin at its nominal rate (|yaw rate| >= 0.9 rad/s) or a backup (speed >= 0.10
m/s); trials that hit the tail of a spin are not exposed. All other faults are triggered on motion (>= 0.12 m/s, reactive: >= 0.30 m/s, dock: BACK
state) and count as exposed.
M8 exposure criteria (written down 2026-11-15, BEFORE any M8 trial was run; identical to the M7 definition above, now also enforced by the
trigger `recovery_nominal`): behavior_server faults are exposed iff the ground truth at the signal shows a spin at >= 0.9 rad/s or a backup at >= 0.10
m/s; controller_server and all other faults are triggered on motion >= 0.12 m/s and count as exposed; dock_detector / dock_drive faults are triggered when
dock_drive is in BACK, the robot moves >= 0.03 m/s, and this has held for 2.0 s (about 4.6 s of the 6.6 s reversing remain), and count as exposed.
Trials the harness reports as invalid (did not arm, no trigger in 150 s, process missing) are not counted at all. "Detected within hold" is the stand-in soft
stop within the 5 s hold, "n exposed" is the count after these criteria; a series is only called complete with >= 10 exposed trials per fault.
Host load per row: median and max of the 1-minute load average (`loadavg_series`, sampled every 10 s) over the trials of the row.
`--latest-label-per-task`: the published tables: per task only the trials of the LAST trial series (label) recorded for it (earlier series had a trigger
that did not hit the intended motion, see docs/FINDINGS.md). `--detail` prints every trial (valid or not) with the invalid reason.
"""

import collections
import json
import math
import statistics
import sys

HOLD_S_DEFAULT = 5.0
EXPOSED_YAW_RATE, EXPOSED_SPEED = 0.9, 0.10


def mmm(v):
    v = [x for x in v if x is not None]
    return "-" if not v else f"{min(v):.2f}/{statistics.median(v):.2f}/{max(v):.2f}"


def exposed(d):
    """See the module docstring. A trial without pre_signal data is not exposed."""
    pre = d.get("pre_signal") or {}
    if not pre:
        return False
    if pre.get("when") in ("recovery", "recovery_nominal"):
        return (pre.get("yaw_rate") or 0.0) >= EXPOSED_YAW_RATE or (pre.get("speed_mps") or 0.0) >= EXPOSED_SPEED
    return True


def detected_within_hold(d):
    t = d.get("standin_soft_stop_s")
    return t is not None and t <= (d.get("hold_s") or HOLD_S_DEFAULT)


def rotation_deg(d):
    """Rotation (degrees) between the signal and standstill from the ground-truth YAW in truth_track ([t_rel, x, y, v, |w|, yaw], samples ~0.25 s apart):
    yaw is unwrapped, interpolated linearly at the signal (t = 0) and at the standstill time, and the absolute yaw changes are summed over the samples in between.
    (M7 and the first M8 tables integrated |w| only up to the last sample before standstill and under-reported by the part of a sample interval at the end.)"""
    st = d.get("standstill_s")
    tt = d.get("truth_track") or []
    if st is None or len(tt) < 2 or st <= 0:
        return None
    ts = [t[0] for t in tt]
    yaw, prev = [], None
    for t in tt:
        y = t[5]
        if prev is not None:
            while y - prev > math.pi:
                y -= 2 * math.pi
            while y - prev < -math.pi:
                y += 2 * math.pi
        yaw.append(y)
        prev = y

    def at(t):
        if t <= ts[0]:
            return yaw[0]
        if t >= ts[-1]:
            return yaw[-1]
        for i in range(1, len(ts)):
            if ts[i] >= t:
                f = (t - ts[i - 1]) / (ts[i] - ts[i - 1]) if ts[i] > ts[i - 1] else 0.0
                return yaw[i - 1] + f * (yaw[i] - yaw[i - 1])
    if ts[0] > 0 or ts[-1] < st:
        return None
    pts = [(0.0, at(0.0))] + [(t, y) for t, y in zip(ts, yaw) if 0.0 < t < st] + [(st, at(st))]
    return round(sum(abs(b[1] - a[1]) for a, b in zip(pts, pts[1:])) * 180 / math.pi, 1)


def load_stats(trials):
    """(median of the per-trial medians, max over all samples) of loadavg_series, or '-' (older records have none)."""
    meds, mx = [], None
    for t in trials:
        v = [x[1] for x in (t.get("loadavg_series") or [])]
        if v:
            meds.append(statistics.median(v))
            mx = max(v) if mx is None else max(mx, max(v))
    return "-" if not meds else f"{statistics.median(meds):.1f}/{mx:.1f}"


def load(paths):
    out = []
    for path in paths:
        out += [json.loads(line) for line in open(path)]
    return out


def select(trials, labels=None, task=None, latest_per_task=False):
    if task:
        trials = [d for d in trials if d.get("task") == task]
    if labels is not None:
        trials = [d for d in trials if d.get("label") in labels]
    if latest_per_task:   # last series = the greatest label (m7_faults < m7_faults_v2) among the task's trials
        last = {}
        for d in trials:
            last[d.get("task")] = max(last.get(d.get("task"), ""), d.get("label") or "")
        trials = [d for d in trials if d.get("label") == last[d.get("task")]]
    return trials


def table(trials):
    rows = collections.OrderedDict()
    for d in trials:
        if d.get("valid"):
            rows.setdefault((d.get("task", "nav_slam"), d["fault"], d.get("profile")), []).append(d)
    out = []
    for (task, fault, prof), ts in rows.items():
        ex = [t for t in ts if exposed(t)]
        blamed = collections.Counter(b for t in ts for b in (t.get("blamed") or []))
        out.append({"task": task, "fault": fault, "profile": prof, "n": len(ts), "exposed": len(ex),
                    "detected": sum(1 for t in ex if t.get("detected")), "detected_in_hold": sum(1 for t in ex if detected_within_hold(t)),
                    "zero_s": mmm([t.get("gated_first_zero_s") for t in ex]), "standstill_s": mmm([t.get("standstill_s") for t in ex]),
                    "distance_m": mmm([t.get("distance_to_standstill_m") for t in ex]),
                    "rotation_deg": mmm([rotation_deg(t) for t in ex]), "agg_fault_s": mmm([t.get("aggregator_first_fault_s") for t in ex]),
                    "not_stopped": sum(1 for t in ex if t.get("robot_end_state") != "stopped"), "blamed": dict(blamed),
                    "resumed": sum(1 for t in ex if t.get("recovered_by_itself")), "load": load_stats(ts)})
    return out


def main():
    args = sys.argv[1:]
    opt = lambda k: next((a.split("=", 1)[1] for a in args if a.startswith(k + "=")), None)  # noqa: E731
    labels = opt("--label")
    trials = select(load([a for a in args if not a.startswith("--")]), labels.split(",") if labels else None, opt("--task"),
                    "--latest-label-per-task" in args)
    if "--detail" in args:
        for d in trials:
            pre = d.get("pre_signal") or {}
            print(d.get("task"), d["fault"], d["rep"], d.get("label"), "valid" if d["valid"] else "INVALID " + str(d.get("invalid_reason")),
                  "exposed" if exposed(d) else "not-exposed", "pre(speed,yaw)", pre.get("speed_mps"), pre.get("yaw_rate"),
                  "zero", d.get("gated_first_zero_s"), "stand", d.get("standstill_s"), "dist", d.get("distance_to_standstill_m"),
                  "rot", rotation_deg(d), "soft", d.get("standin_soft_stop_s"), "resume", d.get("resume_after_s"), "blamed", d.get("blamed"))
    print("task / fault / profile | valid n | exposed | detected (any time) | detected within hold | gated zero s min/med/max | standstill s | "
          "distance m | rotation deg | not stopped at end | blamed | resumed | host load median/max (1-min avg) | aggregator first fault s min/med/max")
    for r in table(trials):
        print(f"{r['task']} / {r['fault']} / {r['profile']} | {r['n']} | {r['exposed']} | {r['detected']} | {r['detected_in_hold']} | {r['zero_s']} | "
              f"{r['standstill_s']} | {r['distance_m']} | {r['rotation_deg']} | {r['not_stopped']} | {r['blamed']} | {r['resumed']} | {r['load']} | {r['agg_fault_s']}")


if __name__ == "__main__":
    main()
