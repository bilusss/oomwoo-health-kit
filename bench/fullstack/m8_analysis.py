#!/usr/bin/env python3
"""M8 recon analysis of probe_trace.py traces (ROS-free; host or container).

  python3 m8_analysis.py chain TRACE...       command chain: /cmd_vel_nav gaps, ages and stop transitions during smoother output
  python3 m8_analysis.py dock TRACE...        dock fix intervals while dock_drive reverses
  python3 m8_analysis.py replay-chain TRACE... [--in-max-age S] [--stop-grace S]    replay the guard core over the recorded messages
  python3 m8_analysis.py replay-dock TRACE... [--fix-max-age S] [--from-reverse-start]
  python3 m8_analysis.py reactive TRACE...     command period of the reactive coverage planner itself (item D)
  python3 m8_analysis.py derive CHAIN_TRACE... --dock DOCK_TRACE...     the bounds of the shipped configs (formulas in derive())

Trace rows are described in probe_trace.py. Times are the probe's host monotonic clock (receive time), so the numbers include the
probe's own scheduling delay (a few ms at normal load). Phase of a /cmd_vel_nav message = the Nav2 action that was active when it
arrived (spin, backup, wait, drive_on_heading, navigate_to_pose; recovery actions win over navigate_to_pose, which is active
during the recovery branch too).
"""

import argparse
import bisect
import collections
import json
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from oomwoo_health_kit.topic_watch import CommandChainGuard, ReverseFixGuard  # noqa: E402

RECOVERY = ("/spin", "/backup", "/wait", "/drive_on_heading")


def load(paths):
    rows = []
    for p in paths:
        rows += [json.loads(line) for line in open(p)]
    rows.sort(key=lambda r: r[0])
    return rows


def pct(v, q):
    v = sorted(v)
    return v[min(len(v) - 1, int(q * len(v)))]


def stats(v):
    if not v:
        return "n=0"
    return (f"n={len(v)} min={min(v):.3f} p50={statistics.median(v):.3f} p95={pct(v, .95):.3f} p99={pct(v, .99):.3f} "
            f"max={max(v):.3f}")


class Phases:
    """Active action per time from the action status arrays (a goal is active while its status is 1 (accepted) .. 3 (canceling))."""

    def __init__(self, rows):
        self.active = {}
        self.events = []   # (t, action, True/False)
        for r in rows:
            if isinstance(r[1], str) and r[1].endswith("/_action/status"):
                name = r[1][:-len("/_action/status")]
                on = any(s[1] in (1, 2, 3) for s in r[2])
                if on != self.active.get(name, False):
                    self.active[name] = on
                    self.events.append((r[0], name, on))



def phase_function(rows):
    ev = Phases(rows).events
    state = {}
    idx = 0

    def at(t):
        nonlocal idx
        while idx < len(ev) and ev[idx][0] <= t:
            state[ev[idx][1]] = ev[idx][2]
            idx += 1
        act = [a for a in RECOVERY if state.get(a)]
        if act:
            return act[0].lstrip("/")
        return "navigate" if state.get("/navigate_to_pose") else "none"
    return at


def is_zero(lin, ang):
    return abs(lin) <= 1e-9 and abs(ang) <= 1e-9


def chain_report(rows):
    out = []
    ph = phase_function(rows)
    nav = [r for r in rows if r[1] == "/cmd_vel_nav"]
    sm = [r for r in rows if r[1] == "/cmd_vel_smoothed"]
    out.append(f"messages: /cmd_vel_nav {len(nav)}, /cmd_vel_smoothed {len(sm)}")
    # 1. period of /cmd_vel_nav messages by phase (consecutive non-zero messages less than 1.5 s apart)
    per = collections.defaultdict(list)
    for a, b in zip(nav, nav[1:]):
        if not is_zero(a[2], a[3]) and not is_zero(b[2], b[3]) and b[0] - a[0] < 1.5:
            per[ph(b[0])].append(b[0] - a[0])
    ph = phase_function(rows)
    out.append("period of consecutive non-zero /cmd_vel_nav messages by phase (s):")
    for k in sorted(per):
        out.append(f"  {k}: {stats(per[k])}")
    # 2. age of the last non-zero input at each non-zero smoothed sample, when the last input message was non-zero
    ph = phase_function(rows)
    ages = collections.defaultdict(list)
    last_nz, last_zero = None, False
    stops = []          # (zero_input_seen, seconds from the last non-zero input to the first zero smoothed sample, phase)
    pending = None      # (t_last_nz_in, zero_in_seen, phase)
    for r in sorted(nav + sm, key=lambda r: r[0]):
        z = is_zero(r[2], r[3])
        if r[1] == "/cmd_vel_nav":
            if z:
                last_zero = True
                if pending:
                    pending = (pending[0], True, pending[2])
            else:
                last_nz, last_zero = r[0], False
                pending = (r[0], False, ph(r[0]))
        else:
            if not z and last_nz is not None and not last_zero:
                ages[ph(r[0])].append(r[0] - last_nz)
            if z and pending:
                stops.append((pending[1], r[0] - pending[0], pending[2]))
                pending = None
    out.append("age of the last non-zero /cmd_vel_nav at non-zero /cmd_vel_smoothed samples, last input non-zero (s):")
    for k in sorted(ages):
        out.append(f"  {k}: {stats(ages[k])}")
    allv = [x for v in ages.values() for x in v]
    out.append(f"  all: {stats(allv)}")
    zs = decel_times(rows)
    out.append("deceleration after a commanded stop (first zero /cmd_vel_nav message -> first zero /cmd_vel_smoothed message), by phase (s):")
    for k in sorted(zs):
        out.append(f"  {k}: {stats(zs[k])}")
    out.append("stop transitions (last non-zero input -> first zero smoothed sample), by phase and whether a zero input message came (s):")
    grp = collections.defaultdict(list)
    for zi, dt, p in stops:
        grp[(p, "zero input" if zi else "NO zero input")].append(dt)
    for k in sorted(grp):
        out.append(f"  {k[0]} / {k[1]}: {stats(grp[k])}")
    return "\n".join(out)


def decel_times(rows):
    """{phase: [seconds from the first zero input message after a non-zero one to the first zero smoothed message]}."""
    ph = phase_function(rows)
    res, pend, last_nz = collections.defaultdict(list), None, None
    for r in rows:
        if r[1] == "/cmd_vel_nav":
            if not is_zero(r[2], r[3]):
                last_nz, pend = r, None
            elif last_nz is not None and pend is None:
                pend = (r[0], ph(last_nz[0]))
        elif r[1] == "/cmd_vel_smoothed" and pend and is_zero(r[2], r[3]):
            res[pend[1]].append(r[0] - pend[0])
            pend = None
    return res


SMOOTHER_LATENCY_S = 0.05    # a /cmd_vel message that follows a /cmd_vel_smoothed message by less than this came through collision_monitor


def reactive_report(traces, max_gap=1.5):
    """Command period of the planner itself (item D). /cmd_vel has several writers (collision_monitor passing the smoother, docking_server,
    the reactive coverage planner) and the message info carries no publisher id, so a /cmd_vel message counts as the PLANNER's when no
    /cmd_vel_smoothed message arrived in the 50 ms before it (the collision monitor republishes within a few ms). Periods are between
    consecutive planner messages less than `max_gap` apart; active time = the time covered by such runs of messages."""
    per, per_nz, active, n_all, n_planner = [], [], 0.0, 0, 0
    for rows in traces:
        sm = [r[0] for r in rows if r[1] == "/cmd_vel_smoothed"]
        cmds = [r for r in rows if r[1] == "/cmd_vel"]
        own = []
        for r in cmds:
            i = bisect.bisect_right(sm, r[0]) - 1
            if not (i >= 0 and r[0] - sm[i] < SMOOTHER_LATENCY_S):
                own.append(r)
        n_all += len(cmds)
        n_planner += len(own)
        for x, y in zip(own, own[1:]):
            d = y[0] - x[0]
            if d < max_gap:
                per.append(d)
                active += d
                if not is_zero(x[2], x[3]) and not is_zero(y[2], y[3]):
                    per_nz.append(d)
    out = [f"/cmd_vel messages {n_all}, attributed to the planner (no /cmd_vel_smoothed in the 50 ms before) {n_planner}",
           f"planner command period, all consecutive pairs < {max_gap} s apart (s): {stats(per)}",
           f"planner command period, both messages non-zero (s): {stats(per_nz)}",
           f"active time covered by those runs: {active / 60:.1f} min"]
    if per:
        out.append(f"share of periods above 0.15 s: {sum(1 for x in per if x > 0.15) / len(per):.3f}; above 0.30 s: {sum(1 for x in per if x > 0.30) / len(per):.4f}")
    return "\n".join(out)


def ceil50(x):
    """Round UP to the next 50 ms."""
    return round(-(-x // 0.05) * 0.05, 2)


def derive(chain_traces, dock_traces):
    """The bounds of the shipped M8 configs from the recon traces (formulas written down before any M8 trial or soak):
      in_max_age_s = ceil_50ms(1.5 x the longest period between consecutive non-zero /cmd_vel_nav messages),
      stop_grace_s = ceil_50ms(1.5 x the longest time from the first zero input message to the first zero smoothed message),
      fix_max_age_s = ceil_50ms(1.5 x the 99th percentile of the gaps between consecutive dock fixes inside reversing episodes)."""
    out = []
    per, dec = [], []
    for rows in chain_traces:
        nav = [r for r in rows if r[1] == "/cmd_vel_nav"]
        per += [b[0] - a[0] for a, b in zip(nav, nav[1:]) if not is_zero(a[2], a[3]) and not is_zero(b[2], b[3]) and b[0] - a[0] < 1.5]
        for v in decel_times(rows).values():
            dec += v
    if per:
        out.append(f"/cmd_vel_nav: {len(per)} periods, max {max(per):.3f} s -> in_max_age_s = {ceil50(1.5 * max(per)):.2f}")
    if dec:
        out.append(f"commanded stops: {len(dec)}, longest zero input -> zero smoothed {max(dec):.3f} s -> stop_grace_s = {ceil50(1.5 * max(dec)):.2f}")
    gaps = []
    for rows in dock_traces:
        eps, fixes, _ = dock_intervals(rows)
        for a, b in eps:
            fx = [f for f in fixes if a <= f <= b]
            gaps += [y - x for x, y in zip(fx, fx[1:])]
    if gaps:
        p99 = pct(gaps, 0.99)
        out.append(f"dock fixes inside reversing episodes: {len(gaps)} gaps, p99 {p99:.3f} s, max {max(gaps):.3f} s -> fix_max_age_s = {ceil50(1.5 * p99):.2f}")
    return "\n".join(out)


def replay_chain(rows, in_max_age, stop_grace):
    """Feed the guard core with the recorded messages; evaluate at every /cmd_vel_smoothed and /odom message. Returns the list of
    not-ok intervals as (t_start, duration, phase, reason)."""
    g = CommandChainGuard(in_max_age_s=in_max_age, stop_grace_s=stop_grace)
    ph = phase_function(rows)
    bad, cur = [], None
    for r in rows:
        t = r[0]
        if r[1] == "/cmd_vel_nav":
            g.on_in(t, abs(r[2]), abs(r[3]))
        elif r[1] == "/cmd_vel_smoothed":
            g.on_out(t, abs(r[2]), abs(r[3]))
        elif r[1] != "/odom":
            continue
        ok, why = g.ok(t)
        if not ok and cur is None:
            cur = [t, t, ph(t), why]
        elif not ok:
            cur[1] = t
        elif cur is not None:
            bad.append((cur[0], cur[1] - cur[0], cur[2], cur[3]))
            cur = None
    if cur:
        bad.append((cur[0], cur[1] - cur[0], cur[2], cur[3]))
    return bad


def dock_intervals(rows, rev_thr=0.02):
    """Reverse episodes from /cmd_vel (held linear x <= -rev_thr) and the dock_pose message times inside and around them."""
    cmds = [r for r in rows if r[1] == "/cmd_vel"]
    fixes = [r[0] for r in rows if r[1] == "/dock_detector/dock_pose"]
    states = [(r[0], r[2]) for r in rows if r[1] == "/dock_drive/state"]
    eps, cur = [], None
    for r in cmds:
        rev = r[2] <= -rev_thr
        if rev and cur is None:
            cur = [r[0], r[0]]
        elif rev:
            cur[1] = r[0]
        elif cur is not None:
            eps.append(tuple(cur))
            cur = None
    if cur:
        eps.append(tuple(cur))
    return eps, fixes, states


def dock_report(traces):
    """Pooled over traces (each trace has its own time base: never merge their rows)."""
    out = []
    pool = collections.defaultdict(list)
    ep_states, st_names, n_fix, n_eps = collections.Counter(), collections.Counter(), 0, 0
    for rows in traces:
        eps, fixes, states = dock_intervals(rows)
        cmds = [r for r in rows if r[1] == "/cmd_vel"]
        n_fix += len(fixes)
        n_eps += len(eps)
        st_names.update(s for _, s in states)

        def state_at(t, states=states):
            cur = None
            for ts, s_ in states:
                if ts > t:
                    break
                cur = s_
            return cur
        for a, b in eps:
            pool["dur"].append(b - a)
            ep_states[state_at(a)] += 1
            fx = [f for f in fixes if a <= f <= b]
            prev = [f for f in fixes if f < a]
            if prev:
                pool["age_start"].append(a - prev[-1])
            if fx:
                pool["first"].append(fx[0] - a)
            pool["gaps"] += [y - x for x, y in zip(fx, fx[1:])]
            last = fx[-1] if fx else (prev[-1] if prev else None)
            if last is not None:
                pool["age_end"].append(b - last)
            # age of the newest fix at every reverse command inside the episode
            for r in cmds:
                if a <= r[0] <= b:
                    i = bisect.bisect_right(fixes, r[0]) - 1
                    if i >= 0:
                        pool["age_all"].append(r[0] - fixes[i])
    out.append(f"traces {len(traces)}, dock_pose messages {n_fix}, reverse episodes (held linear x <= -0.02) {n_eps}")
    out.append(f"state changes seen: {dict(st_names)}; state at the start of a reverse episode: {dict(ep_states)}")
    out.append(f"reverse episode duration (s): {stats(pool['dur'])}")
    out.append(f"gaps between consecutive fixes INSIDE a reverse episode (s): {stats(pool['gaps'])}")
    g = sorted(pool["gaps"])
    out.append(f"  gaps > 0.9 s: {sum(1 for x in g if x > 0.9)} of {len(g)}; gaps > 1.5 s: {sum(1 for x in g if x > 1.5)}")
    out.append(f"age of the last fix at the START of a reverse episode (s): {stats(pool['age_start'])}")
    out.append(f"time from the start of a reverse episode to its first fix (s): {stats(pool['first'])}")
    out.append(f"age of the newest fix at the END of a reverse episode (s): {stats(pool['age_end'])}")
    out.append(f"age of the newest fix at each reverse command (20 Hz samples) (s): {stats(pool['age_all'])}")
    return "\n".join(out)


def cmds_in(rows, a, b):
    return [r for r in rows if r[1] == "/cmd_vel" and a <= r[0] <= b]


def replay_dock(rows, fix_max_age, from_reverse_start=False):
    g = ReverseFixGuard(fix_max_age_s=fix_max_age, from_reverse_start=from_reverse_start)
    bad, cur = [], None
    for r in rows:
        t = r[0]
        if r[1] == "/cmd_vel":
            g.on_cmd(t, r[2])
        elif r[1] == "/dock_detector/dock_pose":
            g.on_fix(t)
        elif r[1] != "/odom":
            continue
        ok, why = g.ok(t)
        if not ok and cur is None:
            cur = [t, t, why]
        elif not ok:
            cur[1] = t
        elif cur is not None:
            bad.append((cur[0], cur[1] - cur[0], cur[2]))
            cur = None
    if cur:
        bad.append((cur[0], cur[1] - cur[0], cur[2]))
    return bad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["chain", "dock", "replay-chain", "replay-dock", "derive", "reactive"])
    ap.add_argument("traces", nargs="+", help="trace files (derive: the chain traces; the dock traces follow --dock)")
    ap.add_argument("--dock", nargs="*", default=[])
    ap.add_argument("--in-max-age", type=float, default=0.35)
    ap.add_argument("--stop-grace", type=float, default=1.0)
    ap.add_argument("--fix-max-age", type=float, default=1.85)
    ap.add_argument("--from-reverse-start", action="store_true")
    a = ap.parse_args()
    if a.cmd == "derive":
        print(derive([load([p]) for p in a.traces], [load([p]) for p in a.dock]))
        return
    traces = [load([p]) for p in a.traces]     # one time base per file: analysed separately, pooled in the report
    print(f"{len(traces)} trace(s), {sum(len(t) for t in traces)} rows, {sum(t[-1][0] - t[0][0] for t in traces if t):.0f} s")
    if a.cmd == "chain":
        for rows, name in zip(traces, a.traces):
            print(f"== {name}")
            print(chain_report(rows))
    elif a.cmd == "dock":
        print(dock_report(traces))
    elif a.cmd == "reactive":
        print(reactive_report(traces))
    elif a.cmd == "replay-chain":
        for rows, name in zip(traces, a.traces):
            bad = replay_chain(rows, a.in_max_age, a.stop_grace)
            print(f"{name}: in_max_age {a.in_max_age} stop_grace {a.stop_grace}: {len(bad)} not-ok intervals")
            for b in bad[:40]:
                print(f"  t+{b[0] - rows[0][0]:.2f}s lasted {b[1]:.2f}s phase {b[2]}: {b[3]}")
    else:
        for rows, name in zip(traces, a.traces):
            bad = replay_dock(rows, a.fix_max_age, a.from_reverse_start)
            eps = dock_intervals(rows)[0]
            print(f"{name}: fix_max_age {a.fix_max_age}: {len(bad)} not-ok intervals in {len(eps)} reverse episodes")
            for b in bad[:40]:
                print(f"  t+{b[0] - rows[0][0]:.2f}s lasted {b[1]:.2f}s: {b[2]}")


if __name__ == "__main__":
    main()
