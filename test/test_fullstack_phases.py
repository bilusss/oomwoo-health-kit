"""Host test of the ROS-free goal/phase attribution used by bench/fullstack/run.py."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "bench", "fullstack"))

import phases as p  # noqa: E402

G = [{"idx": 0, "t0_mono": 100.0, "t1_mono": 120.0}, {"idx": 1, "t0_mono": 120.2, "t1_mono": 140.0},
     {"idx": 2, "t0_mono": 140.1, "t1_mono": 150.0, "artificial_end": True}]


def ph(t, rec=()):
    return p.classify(t, G, list(rec))[:2]


def test_phases_in_order():
    assert ph(99.0) == (None, "between")
    assert ph(101.0) == (0, "start")
    assert ph(110.0) == (0, "en route")
    assert ph(119.5) == (0, "end")
    assert ph(121.5) == (0, "end")  # coasting after goal 0 wins over goal 1's start
    assert ph(123.0) == (1, "start")
    assert ph(130.0) == (1, "en route")
    assert ph(141.0) == (1, "end")
    assert ph(143.0) == (2, "start")
    assert ph(149.9) == (2, "en route")  # artificial end: no end phase
    assert ph(155.0) == (2, "between")


def test_recovery_has_priority_and_a_tail():
    assert ph(110.0, [(108.0, 111.0)]) == (0, "recovery")
    assert ph(111.9, [(108.0, 111.0)]) == (0, "recovery")
    assert ph(112.5, [(108.0, 111.0)]) == (0, "en route")
    assert ph(130.0, [(129.0, None)]) == (1, "recovery")


def test_summary_counts_by_phase_and_component():
    eps = [{"t0": 119.9, "t1": 120.4, "blamed": {"stale_critical": ["drive_supervision"]}, "odom_v": 0.1, "odom_w": 0.0,
            "cmd_age_wall_s": 0.2, "last_cmd_nonzero": False},
           {"t0": 130.0, "t1": 130.1, "blamed": {"missing_critical": ["lidar"]}, "odom_v": 0.2, "odom_w": 0.0,
            "cmd_age_wall_s": 0.01, "last_cmd_nonzero": True}]
    softs = [{"t": 120.1, "reason": "x"}, {"t": 140.0, "reason": "y"}]
    ev, tab = p.summarize(G, [], eps, softs)
    assert tab["soft_stops_by_phase_component"] == {"end / drive_supervision": 1, "end / (no aggregator episode)": 1}
    assert tab["fault_episodes_by_phase_component"] == {"end / drive_supervision": 1, "en route / lidar": 1}
    assert [e["caused_soft_stop"] for e in ev if e["kind"] == "non_emitting_episode"] == [True, False]


def test_soft_stop_classes():
    ep_named = {"t0": 100.0, "t1": 100.5, "blamed": {"stale_critical": ["odom"]}, "states": ["fault", "arming"], "odom_v": 0, "odom_w": 0,
                "cmd_age_wall_s": 0, "last_cmd_nonzero": False}
    ep_arming = {"t0": 110.0, "t1": 110.3, "blamed": {}, "states": ["arming"], "odom_v": 0, "odom_w": 0, "cmd_age_wall_s": 0,
                 "last_cmd_nonzero": False}
    softs = [{"t": 100.2}, {"t": 105.0}, {"t": 110.2}, {"t": 130.0}]
    ev, tab = p.summarize(G, [], [ep_named, ep_arming], softs, gaps=[(104.8, 105.2)])
    cls = [e["class"] for e in ev if e["kind"] == "soft_stop"]
    assert cls == ["detection", "delivery_gap", "unknown", "unknown"]
    assert tab["soft_stops_by_class"] == {"detection": 1, "delivery_gap": 1, "unknown": 2}
