import pytest

from oomwoo_health_kit.topic_watch import (
    EmitLimiter, MotionConsistency, TfTracker, TtlCache, parse_watches, publishers_ok, queued_too_long,
    stamp_level, strip_frame,
)


def count(src_hz, limit_hz, seconds=10.0, jitter=0.0):
    lim = EmitLimiter(limit_hz)
    n = 0
    for i in range(int(seconds * src_hz)):
        t = i / src_hz + (jitter if i % 2 else -jitter)
        n += lim.allow(t + 100.0)
    return n / seconds


def test_limiter_first_always_allowed():
    assert EmitLimiter(20).allow(5.0)


def test_limiter_caps_fast_source_near_limit():
    assert 19.0 <= count(50, 20) <= 21.0


def test_limiter_passes_source_at_the_limit_with_jitter():
    assert count(20, 20, jitter=0.004) >= 19.0  # not every other beat lost


def test_limiter_slow_source_unchanged():
    assert count(5, 20) == pytest.approx(5.0, abs=0.2)


def test_limiter_gap_restarts_schedule_and_backwards_clock_allowed():
    lim = EmitLimiter(10)
    assert lim.allow(0.0)
    assert not lim.allow(0.02)
    assert lim.allow(5.0)  # long gap
    assert not lim.allow(5.01)
    assert lim.allow(1.0)  # clock jumped back


def test_limiter_rejects_nonfinite_and_bad_rate():
    lim = EmitLimiter(10)
    assert not lim.allow(float("nan"))
    for bad in (0, -1, float("inf"), float("nan")):
        with pytest.raises(ValueError):
            EmitLimiter(bad)


def test_stamp_level():
    assert stamp_level(10.0, 9.9, None) == ("ok", "")
    assert stamp_level(10.0, None, None) == ("ok", "")  # no max age configured: stamps are not judged
    assert stamp_level(10.0, 9.8, 0.5) == ("ok", "")
    assert stamp_level(10.0, 9.5, 0.5)[0] == "ok"  # boundary: not older than
    level, detail = stamp_level(10.0, 9.0, 0.5)
    assert level == "degraded" and "1.000" in detail
    assert stamp_level(10.0, float("nan"), 0.5)[0] == "degraded"


def test_stamp_level_missing_zero_or_future_stamp_is_not_ok_when_max_age_set():
    assert stamp_level(10.0, None, 0.5) == ("degraded", "no header stamp")  # zero stamp is passed as None
    assert stamp_level(10.0, 10.5, 0.5)[0] == "degraded"
    assert stamp_level(10.0, 10.005, 0.5)[0] == "ok"  # within clock-jitter tolerance


def test_queued_too_long():
    assert not queued_too_long(None, 20)
    assert not queued_too_long(0.01, 20)
    assert not queued_too_long(0.10, 20)  # 0.05 + 0.05 slack
    assert queued_too_long(0.11, 20)
    assert queued_too_long(1.0, 5)
    assert not queued_too_long(0.2, 20, slack_s=0.2)
    assert queued_too_long(float("nan"), 20)


def test_ttl_cache():
    calls = []
    c = TtlCache(0.5)
    fetch = lambda: calls.append(1) or len(calls)
    assert c.get(0.0, fetch) == 1 and c.get(0.4, fetch) == 1
    assert c.get(0.5, fetch) == 2
    assert c.get(0.1, fetch) == 3  # clock went backwards: refetch


def test_publishers_ok():
    assert publishers_ok(1, 1)[0] and publishers_ok(0, 1)[0]
    ok, detail = publishers_ok(2, 1)
    assert not ok and "2 publishers" in detail


def mc(**kw):
    return MotionConsistency(**kw)


def test_motion_stationary_is_ok_without_commands():
    m = mc()
    m.on_odom(1.0, 0.0, 0.0)
    assert m.ok(1.0)[0]
    m.on_odom(1.1, 0.019, 0.049)  # below both thresholds
    assert m.ok(1.1)[0]


def test_motion_moving_with_fresh_command_is_ok():
    m = mc()
    m.on_odom(1.0, 0.2, 0.0)
    m.on_cmd(0.9)
    assert m.ok(1.0)[0]
    m.on_odom(1.05, 0.0, 0.3)  # rotating counts as moving
    assert m.ok(1.05)[0]


def test_motion_moving_with_stale_or_no_command_is_not_ok():
    m = mc()
    m.on_odom(1.0, 0.2, 0.0)
    ok, detail = m.ok(1.0)
    assert not ok and "without a command" in detail
    m.on_cmd(0.8)
    assert not m.ok(1.0)[0]  # 0.2 s old > 0.15
    m.on_cmd(0.85)
    assert m.ok(1.0)[0]  # exactly at the limit


def test_motion_thresholds():
    m = mc(linear_threshold=0.1, angular_threshold=0.5)
    m.on_odom(1.0, 0.09, 0.4)
    assert m.ok(1.0)[0]
    m.on_odom(1.0, 0.11, 0.0)
    assert not m.ok(1.0)[0]
    m.on_odom(1.0, 0.0, -0.6)
    assert not m.ok(1.0)[0]  # sign does not matter


def test_motion_odom_stale_or_missing_is_not_ok():
    m = mc(odom_max_age_s=0.3)
    assert not m.ok(1.0)[0]
    m.on_odom(1.0, 0.0, 0.0)
    assert m.ok(1.3)[0]
    ok, detail = m.ok(1.31)
    assert not ok and "stale" in detail
    m.on_cmd(1.3)
    assert not m.ok(1.31)[0]  # a command cannot make up for missing odometry


def test_motion_non_finite_values_are_not_ok():
    for bad in (float("nan"), float("inf"), float("-inf")):
        m = mc()
        m.on_cmd(1.0)
        m.on_odom(1.0, bad, 0.0)
        assert not m.ok(1.0)[0]
        m.on_odom(1.0, 0.0, bad)
        assert not m.ok(1.0)[0]
    m = mc()
    m.on_odom(1.0, 0.0, 0.0)
    assert not m.ok(float("nan"))[0]


def test_motion_rejects_bad_parameters():
    for kw in ({"linear_threshold": 0}, {"cmd_max_age_s": -1}, {"odom_max_age_s": float("nan")}):
        with pytest.raises(ValueError):
            mc(**kw)


def test_stamp_level_uses_contract_vocabulary():
    from oomwoo_health_kit.heartbeat import HEALTH_LEVELS

    assert stamp_level(10.0, 1.0, 0.5)[0] in HEALTH_LEVELS


def test_tf_tracker_only_strictly_newer():
    t = TfTracker()
    assert t.is_newer(100)
    assert not t.is_newer(100)
    assert not t.is_newer(99)
    assert t.is_newer(101)


def test_strip_frame():
    assert strip_frame("/map") == "map" and strip_frame("odom") == "odom"


def doc(*watches):
    return {"watches": list(watches)}


def test_parse_defaults():
    (s,) = parse_watches(doc({"component_id": "lidar", "topic": "/scan", "type": "sensor_msgs/msg/LaserScan"}))
    assert s.max_emit_hz == 20.0 and s.stamp_max_age_s is None and not s.is_tf
    assert s.qos == {"reliability": "best_effort", "durability": "volatile", "depth": 10}


def test_parse_tf_watch():
    (s,) = parse_watches(doc({"component_id": "loc", "tf": {"parent": "/map", "child": "odom"}}))
    assert s.is_tf and (s.tf_parent, s.tf_child) == ("map", "odom")
    assert s.topic == "/tf" and s.msg_type == "tf2_msgs/msg/TFMessage"


def test_parse_overrides():
    (s,) = parse_watches(
        doc({"component_id": "a", "topic": "t", "type": "x/msg/Y", "max_emit_hz": 5, "stamp_max_age_s": 0.4,
             "qos": {"reliability": "reliable", "depth": 3}})
    )
    assert s.max_emit_hz == 5.0 and s.stamp_max_age_s == 0.4
    assert s.qos == {"reliability": "reliable", "durability": "volatile", "depth": 3}


@pytest.mark.parametrize(
    "bad",
    [
        None,
        {},
        {"watches": []},
        doc({"topic": "t", "type": "x/msg/Y"}),
        doc({"component_id": "a", "topic": "t"}),
        doc({"component_id": "a", "type": "x/msg/Y"}),
        doc({"component_id": "a", "tf": {"parent": "map"}}),
        doc({"component_id": "a", "tf": {"parent": "m", "child": "o"}, "type": "x/msg/Y"}),
        doc({"component_id": "a", "topic": "t", "type": "x/msg/Y", "max_emit_hz": 0}),
        doc({"component_id": "a", "topic": "t", "type": "x/msg/Y", "stamp_max_age_s": -1}),
        doc({"component_id": "a", "topic": "t", "type": "x/msg/Y", "qos": {"reliability": "maybe"}}),
        doc({"component_id": "a", "topic": "t", "type": "x/msg/Y", "qos": {"depth": 0}}),
        doc({"component_id": "a", "topic": "t", "type": "x/msg/Y", "qos": {"durability": "transient_local"}}),
        doc({"component_id": "a", "topic": "t", "type": "x/msg/Y", "max_publishers": 0}),
        doc({"component_id": "a", "motion_consistency": {"odom_topic": "/odom"}}),
        doc({"component_id": "a", "motion_consistency": {"odom_topic": "/o", "cmd_topic": "/c", "cmd_max_age_s": 0}}),
        doc({"component_id": "a", "topic": "t", "motion_consistency": {"odom_topic": "/o", "cmd_topic": "/c"}}),
        doc({"component_id": "a", "topic": "t", "type": "x/msg/Y", "qos": {"bogus": 1}}),
        doc({"component_id": "a", "topic": "t", "type": "x/msg/Y"}, {"component_id": "a", "topic": "u", "type": "x/msg/Y"}),
    ],
)
def test_parse_rejects(bad):
    with pytest.raises(ValueError):
        parse_watches(bad)


def test_parse_max_publishers_and_motion():
    a, b = parse_watches(doc(
        {"component_id": "a", "topic": "t", "type": "x/msg/Y"},
        {"component_id": "b", "topic": "t", "type": "x/msg/Y", "max_publishers": 6},
    ))
    assert a.max_publishers == 1 and b.max_publishers == 6
    (m,) = parse_watches(doc({"component_id": "m", "motion_consistency": {"odom_topic": "/o", "cmd_topic": "/c"}}))
    assert m.is_motion and m.motion["cmd_max_age_s"] == 0.15 and m.msg_type == "nav_msgs/msg/Odometry"
