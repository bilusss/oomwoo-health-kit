"""ROS-free evaluation of one fault-injection trial (M6c). All times are host monotonic seconds (one clock for the worker, the
recorder process and the gz ground-truth reader); results are relative to the signal time.

Inputs
  t_sig, t_cont  signal time and resume time (SIGCONT / unpause; None for kills)
  rec            recorder dump: ev_out/ev_in [[t, nonzero]], ev_stack [[t, state, blamed]], states, softs, faults
  truth          gz ground truth [(t, x, y, v, w)] from `gz topic -e -t /odom` (physics pose/twist, independent of the ROS bridge)
  t_end          end of the observation
"""

import math

STOP_V = 0.01       # m/s
STOP_W = 0.02       # rad/s
STOP_HOLD_S = 0.5   # speed must stay below the limits this long
SILENT_S = 0.5      # the truth stream is "silent" (physics paused or gz gone) after this long without a message


def _first(rows, t0, pred):
    for r in rows:
        if r[0] >= t0 and pred(r):
            return r
    return None


def standstill(truth, t_sig, t_end):
    """(time of standstill or None, 'stopped'|'frozen'|'moving'). frozen = the stream went silent while the robot was still moving."""
    rows = [r for r in truth if t_sig <= r[0] <= t_end]
    start = None
    for r in rows:
        if r[3] < STOP_V and abs(r[4]) < STOP_W:
            start = r[0] if start is None else start
            if r[0] - start >= STOP_HOLD_S:
                return start, "stopped"
        else:
            start = None
    if rows:
        last = rows[-1]
        if t_end - last[0] > SILENT_S and (last[3] >= STOP_V or abs(last[4]) >= STOP_W):
            return last[0], "frozen"
        if start is not None and last[0] - start < STOP_HOLD_S and t_end - last[0] > SILENT_S:
            return start, "frozen"
    return None, "moving"


def path_length(truth, t0, t1):
    pts = [(r[1], r[2]) for r in truth if t0 <= r[0] <= t1]
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]) if math.hypot(b[0] - a[0], b[1] - a[1]) < 0.5)


def evaluate(t_sig, t_cont, t_end, rec, truth, clock):
    out = {}
    rel = lambda t: None if t is None else round(t - t_sig, 3)  # noqa: E731
    v_sig = next((r for r in reversed(truth) if r[0] <= t_sig), None)
    out["speed_at_signal_mps"] = None if v_sig is None else round(v_sig[3], 3)
    # --- commands
    z = _first(rec.get("ev_out", []), t_sig, lambda r: not r[1])
    out["gated_first_zero_s"] = rel(None if z is None else z[0])
    nz_before = [r for r in rec.get("ev_out", []) if t_sig <= r[0] and (z is None or r[0] < z[0]) and r[1]]
    out["gated_nonzero_msgs_before_zero"] = len(nz_before)
    out["gated_last_nonzero_before_zero_s"] = rel(nz_before[-1][0]) if nz_before else None
    lim = t_cont if t_cont is not None else t_end
    out["gated_nonzero_after_zero_before_resume"] = (
        0 if z is None else sum(1 for r in rec.get("ev_out", []) if z[0] < r[0] <= lim and r[1]))
    last_in = [r for r in rec.get("ev_in", []) if t_sig <= r[0] <= lim and r[1]]
    out["cmd_in_last_nonzero_s"] = rel(last_in[-1][0]) if last_in else None
    out["cmd_in_nonzero_msgs_after_signal"] = len(last_in)
    # --- aggregator and stand-in
    fs = _first(rec.get("ev_stack", []), t_sig, lambda r: r[1] == "fault")
    out["aggregator_first_fault_s"] = rel(None if fs is None else fs[0])
    out["blamed_first_sample"] = None if fs is None else fs[2]
    ep = next((e for e in rec.get("faults", []) if e["t0"] >= t_sig - 0.05), None)
    out["blamed"] = None if ep is None else sorted(set(sum((ep["blamed"][k] for k in ep["blamed"]), [])))
    out["blamed_detail"] = None if ep is None else ep["blamed"]
    out["fault_ctx"] = None if ep is None else {k: ep[k] for k in ("odom_v", "odom_w", "cmd_age_wall_s", "last_cmd_nonzero")}
    ss = _first([[e["t"], e] for e in rec.get("softs", [])], t_sig, lambda r: True)
    out["standin_soft_stop_s"] = rel(None if ss is None else ss[0])
    out["standin_soft_stop_reason"] = None if ss is None else ss[1].get("reason")
    out["standin_transitions"] = [{"t_s": rel(e["t"]), "state": e["state"], "reason": e["reason"]}
                                  for e in rec.get("states", []) if t_sig - 0.05 <= e["t"] <= t_end][:14]
    out["aggregator_changes"] = [{"t_s": rel(e[0]), "state": e[1], "blamed": e[2]} for e in rec.get("ev_stack", [])
                                 if t_sig - 0.05 <= e[0] <= t_end][:14]
    # --- ground truth
    t_stand, how = standstill(truth, t_sig, t_end)
    out["robot_end_state"] = how
    out["standstill_s"] = rel(t_stand)
    d_stand = path_length(truth, t_sig, t_stand if t_stand is not None else t_end)
    out["distance_to_standstill_m"] = round(d_stand, 3)
    out["distance_total_to_end_m"] = round(path_length(truth, t_sig, t_end), 3)
    out["truth_msgs_after_signal"] = sum(1 for r in truth if t_sig <= r[0] <= t_end)
    # --- clock
    pre = [c for c in clock if c[0] <= t_sig]
    post = [c for c in clock if c[0] > t_sig]
    if pre and post:
        adv = post[-1][1] - pre[-1][1]
        out["sim_clock_advance_signal_to_end_s"] = round(adv, 3)
        out["sim_clock_advance_during_hold_s"] = (None if t_cont is None else round(
            (next((c[1] for c in post if c[0] >= t_cont), post[-1][1])) - pre[-1][1], 3))
    # --- recovery by itself (stack-level): stand-in back in RUN and cmd_vel_gated nonzero again after the resume (or after the kill)
    t0 = t_cont if t_cont is not None else t_sig
    back = _first(rec.get("ev_out", []), t0, lambda r: r[1] and z is not None and r[0] > z[0])
    run_after = _first([[e["t"], e] for e in rec.get("states", [])], t0, lambda r: r[1]["state"] == "run")
    out["recovered_gated_nonzero_s"] = None if back is None else round(back[0] - t0, 3)
    out["recovered_standin_run_s"] = None if run_after is None else round(run_after[0] - t0, 3)
    # recovered = commands reach the gated output again after the resume (kills: after the signal). The stand-in can be back in
    # RUN much earlier (e.g. while Nav2 is still stopped: a stationary robot satisfies drive_supervision), see standin_run_from_signal_s.
    out["recovered_by_itself"] = back is not None
    first_soft = out["standin_soft_stop_s"]
    r2 = None if ss is None else _first([[e["t"], e] for e in rec.get("states", [])], ss[0] + 1e-6, lambda r: r[1]["state"] == "run")
    out["standin_run_from_signal_s"] = None if r2 is None else rel(r2[0])
    return out
