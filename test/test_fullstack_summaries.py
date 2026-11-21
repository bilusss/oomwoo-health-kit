"""summarize.py (--by-task, category, active time) and summarize_faults.py on synthetic records."""

import datetime as dt
import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "bench" / "fullstack"))
import summarize as sm  # noqa: E402
import summarize_faults as sf  # noqa: E402

T0 = dt.datetime(2026, 11, 14, 12, 0, 0).astimezone()


def iso(sec):
    return (T0 + dt.timedelta(seconds=sec)).isoformat(timespec="milliseconds")


def session(task="coverage", profile="default", sim_s=1800.0, events=(), stops=(), goals=(), valid=True):
    health = []
    for t, v, age in stops:   # soft stops with a context
        health.append({"kind": "soft_stop", "wall": iso(t), "blamed": ["drive_supervision"],
                       "fault_ctx": {"odom_v": v, "odom_w": 0.0, "cmd_age_wall_s": age, "last_cmd_nonzero": True}})
    return {"task": task, "profile": profile, "valid": valid, "sim_s": sim_s, "window_start_wall": iso(0), "distance_m": 10.0,
            "soft_stops_n": len(stops), "hb_gaps_n": 0, "uncommanded_motion": {"n": len(events), "events": [
                {"wall": iso(t), "duration_s": 0.06} for t in events]}, "health_events": health, "goals": list(goals),
            "health_event_tables": {"soft_stops_by_class": {"detection": len(stops)}}, "action_tally": {}}


def test_active_seconds_ends_at_last_motion_evidence_for_coverage():
    d = session(events=[10.0, 500.0, 990.0], stops=[(1500.0, 0.0, 900.0)])     # the stop at 1500 s is an idle stop (still robot, old command)
    assert abs(sm.active_seconds(d) - 990.06) < 1e-6
    d = session(events=[10.0], stops=[(300.0, 0.35, 0.29)])                    # a stop with the robot at 0.35 m/s is activity
    assert abs(sm.active_seconds(d) - 300.0) < 1e-6
    assert sm.active_seconds(session(events=[])) == 0.0
    assert sm.active_seconds(session(events=[5000.0])) == 1800.0               # never above the window


def test_continuous_tasks_are_active_for_the_whole_window():
    assert sm.active_seconds(session(task="nav_amcl", sim_s=1800.0)) == 1800.0


def test_every_coverage_task_idles_after_the_sweep():
    """M9 review: the *_inode coverage tasks were missing, so their whole window counted as active (1.5 h instead of 0.81 / 0.35 h)."""
    import tasks
    cov = {name for name, cfg in tasks.TASKS.items() if cfg.get("workload") == "coverage"}
    assert cov and cov <= set(sm.COVERAGE_TASKS)
    for name in cov:
        assert abs(sm.active_seconds(session(task=name, events=[10.0, 990.0])) - 990.06) < 1e-6


def test_harness_window_is_forward_only():
    a = T0.timestamp() + 100.0
    assert sm.harness_related(iso(100.25), [a]) and sm.harness_related(iso(100.5), [a])
    assert not sm.harness_related(iso(99.9), [a]) and not sm.harness_related(iso(101.2), [a])


def test_category():
    assert sm.category({"pose": "DOCK:center", "result": "succeeded"}) == "dock_succeeded"
    assert sm.category({"pose": "DOCK:away", "result": "canceled", "note": "session ended"}) == "cut_by_session_end"
    assert sm.category({"pose": "COVERAGE", "result": "canceled"}) == "coverage_run_cut_by_session_end"
    assert sm.category({"pose": "P2", "result": "aborted", "scenario": {"wall": "persistent"}}) == "recovery_persistent_aborted"
    assert sm.category({"pose": "P2", "result": "succeeded", "scenario": {"wall": None}}) == "recovery_no_wall_succeeded"
    assert sm.category({"pose": "UNREACH", "result": "aborted"}) == "unplannable"


def test_by_task_rates_use_active_hours(tmp_path):
    d1 = session(events=[10.0, 900.0], stops=[(100.0, 0.35, 0.29), (200.0, 0.35, 0.29), (1700.0, 0.0, 1200.0)])
    d2 = session(valid=False)
    d3 = session(task="dock", goals=[{"pose": "DOCK:center", "result": "succeeded", "harness_actions": [{"what": "set_pose", "epoch": T0.timestamp() + 50.0}]}],
                 stops=[(50.25, 0.0, 0.02), (400.0, 0.0, 0.02)])
    p = tmp_path / "s.jsonl"
    p.write_text("".join(json.dumps(x) + "\n" for x in (d1, d2, d3)))
    rows = sm.aggregate([str(p)])
    r = rows[("coverage", "default")]
    assert r["sessions"] == 2 and r["invalid"] == 1 and r["soft_active"] == 2 and r["soft_idle"] == 1
    assert abs(r["active_h"] - 900.06 / 3600) < 1e-6 and abs(r["sim_h"] - 0.5) < 1e-9
    dk = rows[("dock", "default")]
    assert dk["soft_after_harness"] == 1 and dk["soft_other"] == 1 and abs(dk["active_h"] - 0.5) < 1e-9


def trial(fault="sigstop_behavior", when="recovery", yaw=0.96, speed=0.0, soft=None, label="a", task="recovery", track=None, standstill=1.0):
    return {"task": task, "fault": fault, "rep": 1, "profile": "default", "label": label, "valid": True, "hold_s": 5.0,
            "pre_signal": {"when": when, "yaw_rate": yaw, "speed_mps": speed}, "standin_soft_stop_s": soft, "detected": soft is not None,
            "gated_first_zero_s": 1.2, "standstill_s": standstill, "distance_to_standstill_m": 0.0, "robot_end_state": "stopped",
            "truth_track": track or [[-0.3, 0, 0, 0, 1.0, -0.3], [-0.05, 0, 0, 0, 1.0, -0.05], [0.2, 0, 0, 0, 1.0, 0.2], [0.7, 0, 0, 0, 1.0, 0.7],
                                     [1.1, 0, 0, 0, 0.0, 0.9], [1.4, 0, 0, 0, 0.0, 0.9]]}


def test_exposure_and_hold_detection():
    assert sf.exposed(trial(yaw=0.96)) and not sf.exposed(trial(yaw=0.71)) and sf.exposed(trial(yaw=0.0, speed=0.15))
    assert sf.exposed(trial(when="moving", yaw=0.0)) and not sf.exposed({})
    assert sf.detected_within_hold(trial(soft=0.4)) and not sf.detected_within_hold(trial(soft=8.4)) and not sf.detected_within_hold(trial())


def test_rotation_is_the_ground_truth_yaw_change_from_the_signal_to_standstill():
    # yaw 0.0 at the signal (t=0, interpolated between -0.05 and 0.2), 0.9 rad at standstill t=1.1: 51.6 degrees
    assert abs(sf.rotation_deg(trial(standstill=1.1)) - 51.6) < 0.2
    # standstill between two samples: interpolated (0.7 -> 1.1 goes 0.7 -> 0.9 rad; at 0.9 the yaw is 0.8 rad)
    assert abs(sf.rotation_deg(trial(standstill=0.9)) - 45.8) < 0.2
    # the old function stopped at the last sample before standstill (0.7 rad = 40 deg) and under-reported
    assert sf.rotation_deg(trial(standstill=0.9)) > 40.2
    assert sf.rotation_deg({"standstill_s": None, "truth_track": []}) is None


def test_rotation_unwraps_yaw_across_pi():
    tr = [[-0.3, 0, 0, 0, 1.0, 3.0], [-0.05, 0, 0, 0, 1.0, 3.05], [0.2, 0, 0, 0, 1.0, -3.03], [0.7, 0, 0, 0, 1.0, -2.5], [1.1, 0, 0, 0, 0.0, -2.4]]
    # 3.05 -> 3.253 (= -3.03 + 2 pi) at 0.2 ... at t=0 yaw 3.0 + ... total from signal to 1.1: 3.8832 - ... check against the unwrapped difference
    assert abs(sf.rotation_deg(trial(track=tr, standstill=1.1)) - ((-2.4 + 2 * 3.141592653589793) - (3.05 + (3.253185307 - 3.05) * 0.05 / 0.25)) * 180 / 3.141592653589793) < 0.2


def test_table_counts_only_exposed_and_latest_label():
    ts = [trial(label="m7_faults", yaw=0.96), trial(label="m7_faults_v2", yaw=0.96, soft=8.4), trial(label="m7_faults_v2", yaw=0.3, soft=8.4)]
    sel = sf.select(ts, latest_per_task=True)
    assert len(sel) == 2
    (row,) = sf.table(sel)
    assert row["n"] == 2 and row["exposed"] == 1 and row["detected"] == 1 and row["detected_in_hold"] == 0
    assert sf.select(ts, labels=["m7_faults"]) == [ts[0]] and sf.select(ts, task="nope") == []
