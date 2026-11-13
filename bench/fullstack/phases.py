"""ROS-free attribution of health events (aggregator fault episodes, stand-in soft stops) to goals and goal phases (M6c).

All times are one clock (host monotonic). Phase of an event at time t, first match wins:
  recovery  a Nav2 recovery action (spin, backup, wait, drive_on_heading) was active at t or ended less than RECOVERY_TAIL_S before
  end       t is in [t1 - END_HEAD_S, t1 + END_TAIL_S] of a goal (t1 = result received; the previous goal wins over the next goal's
            start, because the robot is still coasting then and the next goal is sent within ~0.1 s)
  start     t is in [t0, t0 + START_S] of a goal (t0 = goal sent: planning, first commands, acceleration)
  en route  inside a goal otherwise
  between   outside every goal (before the first, after the last, or in a gap longer than END_TAIL_S)
A goal cancelled by the end of the session has an artificial end (no end phase). The windows are fixed constants, not tuned; every event also carries the raw offsets so it can be re-bucketed.
"""

START_S = 3.0
END_HEAD_S = 1.0
END_TAIL_S = 2.0
RECOVERY_TAIL_S = 1.0
PHASES = ("start", "en route", "end", "recovery", "between")


def classify(t, goals, recoveries):
    """-> (goal_index or None, phase, since_goal_start_s, to_goal_end_s). goals: dicts with t0_mono/t1_mono sorted by t0."""
    cur = None
    for i, g in enumerate(goals):
        if g["t0_mono"] <= t:
            cur = i
        else:
            break
    since = None if cur is None else round(t - goals[cur]["t0_mono"], 3)
    to_end = None if cur is None else round(goals[cur]["t1_mono"] - t, 3)
    if cur is None:
        return None, "between", since, to_end
    if any(a <= t <= (b if b is not None else float("inf")) + RECOVERY_TAIL_S for a, b in recoveries):
        return cur, "recovery", since, to_end
    if cur > 0:
        p = goals[cur - 1]
        if not p.get("artificial_end") and p["t1_mono"] - END_HEAD_S <= t <= p["t1_mono"] + END_TAIL_S:
            return cur - 1, "end", round(t - p["t0_mono"], 3), round(p["t1_mono"] - t, 3)
    g = goals[cur]
    if not g.get("artificial_end") and g["t1_mono"] - END_HEAD_S <= t <= g["t1_mono"] + END_TAIL_S:
        return cur, "end", since, to_end
    if t <= g["t0_mono"] + START_S:
        return cur, "start", since, to_end
    if t <= g["t1_mono"]:
        return cur, "en route", since, to_end
    return cur, "between", since, to_end


def attach_episode(t_soft, episodes):
    """The aggregator fault episode that explains a stand-in soft stop at t_soft: the latest one that started no later than
    50 ms after it (delivery skew) and was still open or ended less than 0.6 s before (0.25 s arming + margin), else None."""
    best = None
    for ep in episodes:
        if ep["t0"] <= t_soft + 0.05 and (ep["t1"] is None or t_soft <= ep["t1"] + 0.6):
            best = ep
    return best


def blamed_components(ep):
    if ep is None:
        return ["(no aggregator fault)"]
    b = ep["blamed"]
    out = sorted(set(b.get("missing_critical", [])) | set(b.get("stale_critical", [])) | set(b.get("unhealthy_critical", [])))
    return out or ["(fault without component)"]


def summarize(goals, recoveries, episodes, softs):
    """goals: records with t0_mono/t1_mono; recoveries: [(a, b)]; episodes: recorder fault episodes; softs: recorder soft stops.
    Returns (events, tables). tables: counts by phase and 'phase / component' for soft stops and for fault episodes."""
    gl = sorted((g for g in goals if "t0_mono" in g), key=lambda g: g["t0_mono"])
    events, soft_by_ph, soft_by_pc, ep_by_pc = [], {}, {}, {}
    attached = set()
    for e in softs:
        ep = attach_episode(e["t"], episodes)
        if ep is not None:
            attached.add(id(ep))
        gi, ph, since, to_end = classify(e["t"], gl, recoveries)
        comps = blamed_components(ep)
        events.append({"kind": "soft_stop", "t_mono": e["t"], "goal": None if gi is None else gl[gi]["idx"], "phase": ph,
                       "since_goal_start_s": since, "to_goal_end_s": to_end, "blamed": comps, "reason": e.get("reason"),
                       "fault_ctx": None if ep is None else {k: ep[k] for k in ("odom_v", "odom_w", "cmd_age_wall_s", "last_cmd_nonzero")}})
        soft_by_ph[ph] = soft_by_ph.get(ph, 0) + 1
        for c in comps:
            soft_by_pc[f"{ph} / {c}"] = soft_by_pc.get(f"{ph} / {c}", 0) + 1
    for ep in episodes:
        gi, ph, since, to_end = classify(ep["t0"], gl, recoveries)
        comps = blamed_components(ep)
        for c in comps:
            ep_by_pc[f"{ph} / {c}"] = ep_by_pc.get(f"{ph} / {c}", 0) + 1
        events.append({"kind": "fault_episode", "t_mono": ep["t0"], "goal": None if gi is None else gl[gi]["idx"], "phase": ph,
                       "since_goal_start_s": since, "to_goal_end_s": to_end, "blamed": comps,
                       "duration_ms": None if ep["t1"] is None else round((ep["t1"] - ep["t0"]) * 1000, 1),
                       "caused_soft_stop": id(ep) in attached, "odom_v": ep["odom_v"], "odom_w": ep["odom_w"],
                       "cmd_age_wall_s": ep["cmd_age_wall_s"], "last_cmd_nonzero": ep["last_cmd_nonzero"]})
    events.sort(key=lambda x: x["t_mono"])
    tables = {"soft_stops_by_phase": soft_by_ph, "soft_stops_by_phase_component": soft_by_pc,
              "fault_episodes_by_phase_component": ep_by_pc, "fault_episodes_n": len(episodes), "soft_stops_n": len(softs)}
    return events, tables
