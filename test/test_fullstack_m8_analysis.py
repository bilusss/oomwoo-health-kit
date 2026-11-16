"""ROS-free tests of the M8 recon analysis (bench/fullstack/m8_analysis.py) on synthetic traces, and the pre-registered bounds."""

import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "bench" / "fullstack"))
import m8_analysis as a  # noqa: E402


def stream(t0, t1, hz, topic, lin=0.2, ang=0.0):
    n = int(round((t1 - t0) * hz))
    return [[t0 + i / hz, topic, lin, ang] for i in range(n)]


def test_replay_chain_flags_a_replayed_setpoint_after_the_commander_died():
    rows = stream(0, 10, 5, "/cmd_vel_nav", 0.2) + stream(0, 12, 20, "/cmd_vel_smoothed", 0.2) + stream(0, 12, 50, "/odom", 0.0)
    rows.sort(key=lambda r: r[0])
    bad = a.replay_chain(rows, 0.35, 0.55)
    # last input at 9.8; output goes on until 12.0 (replayed): not ok from 9.8 + 0.35
    assert len(bad) == 1 and abs(bad[0][0] - 10.16) < 0.05 and bad[0][1] > 1.5


def test_replay_chain_accepts_a_commanded_stop_with_deceleration():
    rows = stream(0, 5, 5, "/cmd_vel_nav", 0.2) + [[5.0, "/cmd_vel_nav", 0.0, 0.0]]
    rows += stream(0, 5.3, 20, "/cmd_vel_smoothed", 0.2) + [[5.3, "/cmd_vel_smoothed", 0.0, 0.0]] + stream(0, 8, 50, "/odom", 0.0)
    rows.sort(key=lambda r: r[0])
    assert a.replay_chain(rows, 0.35, 0.55) == []


def test_replay_dock_flags_a_dead_detector_and_accepts_a_missed_fix():
    cmd = stream(0.02, 8, 20, "/cmd_vel", -0.06)
    fixes = [[i * 0.6 - 0.01, "/dock_detector/dock_pose", 0.0, -0.2, 0.0] for i in range(14)]
    odom = stream(0, 9, 50, "/odom", 0.0)
    rows = sorted(cmd + fixes + odom, key=lambda r: r[0])
    assert a.replay_dock(rows, 1.8) == []
    del_gap = [r for r in rows if not (r[1].endswith("dock_pose") and abs(r[0] - 3.0) < 1e-6)]   # one missed fix: a 1.2 s gap
    assert a.replay_dock(del_gap, 1.8) == []
    dead = [r for r in rows if not (r[1].endswith("dock_pose") and r[0] > 3.0)]
    bad = a.replay_dock(dead, 1.8)
    assert len(bad) == 1 and abs(bad[0][0] - (3.0 - 0.01 + 1.8)) < 0.06


def test_dock_intervals_find_reverse_episodes():
    cmd = stream(0, 2, 20, "/cmd_vel", 0.1) + stream(2, 5, 20, "/cmd_vel", -0.06) + stream(5, 6, 20, "/cmd_vel", 0.0)
    eps, fixes, states = a.dock_intervals(cmd)
    assert len(eps) == 1 and abs(eps[0][0] - 2.0) < 1e-9 and abs(eps[0][1] - 4.95) < 1e-9 and fixes == [] and states == []


def test_shipped_bounds_are_what_the_retained_recon_traces_give():
    """The configs' bounds are derived (formulas in m8_analysis.derive) from the retained recon traces; this fails if either changes."""
    import pytest
    d = ROOT / "results" / "fullstack" / "m8_recon"
    chain, dock = [d / "trace_recovery_1.jsonl"], sorted(d.glob("trace_dock_?.jsonl"))
    if not (chain[0].exists() and dock):
        pytest.skip("recon traces not present")
    out = a.derive([a.load([str(p)]) for p in chain], [a.load([str(p)]) for p in dock])
    assert "in_max_age_s = 0.35" in out and "stop_grace_s = 0.55" in out and "fix_max_age_s = 1.85" in out, out


def test_reactive_report_separates_the_planner_from_the_smoother_chain():
    sm = stream(0, 5, 20, "/cmd_vel_smoothed", 0.2)
    via_monitor = [[r[0] + 0.003, "/cmd_vel", r[2], r[3]] for r in sm]           # collision_monitor republishes within 3 ms
    planner = [[10 + i * 0.2 + (0.2 if i >= 10 else 0), "/cmd_vel", 0.3, 0.0] for i in range(30)]   # 5 Hz, one 0.4 s gap
    out = a.reactive_report([sorted(sm + via_monitor + planner, key=lambda r: r[0])])
    assert "attributed to the planner (no /cmd_vel_smoothed in the 50 ms before) 30" in out
    assert "max=0.400" in out and "n=29" in out


def test_collect_routes_every_record_somewhere(tmp_path):
    import json
    import collect_m8 as c
    src, out = tmp_path / "m8", tmp_path / "out"
    (src / "1").mkdir(parents=True)
    (src / "1" / "faults.jsonl").write_text("\n".join(json.dumps(d) for d in (
        {"label": "m8_faults", "task": "recovery_probe_chain"}, {"label": "m8_faults", "task": "recovery_chain"},
        {"label": "m8_faults_control", "task": "recovery_probe"}, {"label": "m8_smoke", "task": "x"}, {"label": "zzz", "task": "x"})) + "\n")
    (src / "1" / "sessions.jsonl").write_text("\n".join(json.dumps(d) for d in (
        {"label": "m8_soak", "task": "dock_fix"}, {"label": "m8_soak", "task": "recovery_chain"}, {"label": "m8_recon", "task": "dock"})) + "\n")
    n = c.collect(str(src), str(out), "T")
    assert sum(n.values()) == 8
    assert n["faults_m8_T.jsonl"] == 2 and n["soak_m8_dock_fix_T.jsonl"] == 1 and n["soak_m8_recovery_chain_T.jsonl"] == 1
    assert n["m8_recon/sessions_recon.jsonl"] == 1 and n["_m8_smoke/records_smoke.jsonl"] == 2 and n["_m8_smoke/records_other.jsonl"] == 1
