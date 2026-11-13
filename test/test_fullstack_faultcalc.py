"""Host test of the ROS-free fault-trial evaluation used by bench/fullstack/run.py (faults mode)."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "bench", "fullstack"))

import faultcalc as f  # noqa: E402


def truth_decel(t_sig, v0=0.3, decel=1.0, hz=50, until=3.0, x0=0.0):
    rows, x, v, t = [], x0, v0, t_sig - 1.0
    while t < t_sig + until:
        if t >= t_sig + 0.3:
            v = max(0.0, v - decel / hz)
        x += v / hz
        rows.append((t, x, 0.0, v, 0.0))
        t += 1.0 / hz
    return rows


def test_sigstop_trial_numbers():
    t_sig, t_cont, t_end = 100.0, 106.0, 120.0
    rec = {"ev_out": [[100.0 + i * 0.05, True] for i in range(-10, 7)] + [[100.35, False]] + [[107.0 + i * 0.05, True] for i in range(5)],
           "ev_in": [[100.0 + i * 0.05, True] for i in range(-10, 1)],
           "ev_stack": [[100.1, "fault", {"stale_critical": ["drive_supervision"]}], [106.2, "healthy", {}]],
           "faults": [{"t0": 100.1, "t1": 106.2, "blamed": {"stale_critical": ["drive_supervision", "lidar"]}, "odom_v": 0.3,
                       "odom_w": 0.0, "cmd_age_wall_s": 0.1, "last_cmd_nonzero": True}],
           "states": [{"t": 100.34, "state": "soft_stop", "reason": "no heartbeat"}, {"t": 106.4, "state": "run", "reason": "3 hb"}],
           "softs": [{"t": 100.34, "reason": "no heartbeat"}]}
    r = f.evaluate(t_sig, t_cont, t_end, rec, truth_decel(t_sig), [(99.0, 50.0), (101.0, 51.0), (119.0, 69.0)])
    assert r["gated_first_zero_s"] == 0.35
    assert r["blamed"] == ["drive_supervision", "lidar"]
    assert r["standin_soft_stop_s"] == 0.34
    assert r["robot_end_state"] == "stopped" and 0.3 < r["standstill_s"] < 1.5
    assert 0.0 < r["distance_to_standstill_m"] < 0.3
    assert r["recovered_by_itself"] and r["standin_run_from_signal_s"] == 6.4


def test_frozen_truth_stream_is_not_a_stop():
    t_sig = 50.0
    truth = [(t_sig - 1 + i * 0.02, i * 0.006, 0.0, 0.3, 0.0) for i in range(100)]  # moving, then silent after t_sig + 1
    r = f.evaluate(t_sig, 56.0, 70.0, {}, truth, [])
    assert r["robot_end_state"] == "frozen"


def test_robot_never_stops():
    t_sig = 10.0
    truth = [(t_sig + i * 0.02, i * 0.006, 0.0, 0.3, 0.0) for i in range(500)]
    r = f.evaluate(t_sig, None, t_sig + 9.9, {}, truth, [])
    assert r["robot_end_state"] == "moving" and r["standstill_s"] is None and r["distance_total_to_end_m"] > 2.5
