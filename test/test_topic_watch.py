import pytest

from oomwoo_health_kit.topic_watch import (
    CommandChainGuard, EmitLimiter, MotionConsistency, ReverseFixGuard, TfTracker, TtlCache, parse_watches, publishers_ok, queued_too_long,
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


def quiet(m, t0, t1, lin=0.0, ang=0.0, hz=50):
    """odom samples at hz from t0 to t1 (inclusive)"""
    n = int(round((t1 - t0) * hz))
    for i in range(n + 1):
        m.on_odom(t0 + i / hz, lin, ang)


def test_motion_no_command_ever_needs_settle_time():
    m = mc()
    m.on_odom(1.0, 0.0, 0.0)
    assert not m.ok(1.0)[0]  # just started being quiet
    quiet(m, 1.0, 1.1)
    assert m.ok(1.1)[0]  # settle_s 0.1 reached
    m.on_odom(1.11, 0.019, 0.049)  # still below both thresholds
    assert m.ok(1.11)[0]


def test_motion_no_command_ever_and_moving_is_not_ok():
    m = mc()
    quiet(m, 1.0, 1.5, lin=0.2)
    ok, detail = m.ok(1.5)
    assert not ok and "moving" in detail


def test_motion_fresh_command_beats_whatever_odom_says():
    m = mc()
    m.on_odom(1.0, 0.2, 0.0)
    m.on_cmd(0.9, 0.2)
    assert m.ok(1.0)[0]
    m.on_odom(1.05, 0.0, 0.3)  # rotating counts as moving, command is fresh
    assert m.ok(1.05)[0]
    m2 = mc()
    m2.on_odom(1.0, 0.0, 0.0)  # odom reads zero but a fresh command is there (robot about to move)
    m2.on_cmd(1.0, 0.2)
    assert m2.ok(1.0)[0]


def test_motion_nonzero_setpoint_without_fresh_command_never_beats():
    m = mc()
    m.on_cmd(0.8, 0.2)  # last command 0.2 s old > 0.15
    m.on_odom(1.0, 0.2, 0.0)
    ok, detail = m.ok(1.0)
    assert not ok and "non-zero setpoint" in detail
    m.on_cmd(0.85, 0.2)
    assert m.ok(1.0)[0]  # exactly at the limit


def test_motion_creep_command_below_odom_threshold_dies_with_commander():
    m = mc()
    m.on_cmd(1.0, 0.015)  # held creep command, below the 0.02 m/s odom threshold
    quiet(m, 1.0, 5.0, lin=0.015)  # odometry below the threshold the whole time
    assert not m.ok(5.0)[0]
    assert not m.ok(60.0)[0]


def test_motion_odom_reads_zero_with_nonzero_setpoint():
    m = mc()
    m.on_cmd(1.0, 0.2)
    quiet(m, 1.0, 3.0)  # stalled wheels / frozen encoder: alive odometry reads zero
    assert not m.ok(3.0)[0]
    m.on_cmd(3.0, 0.2)
    assert m.ok(3.0)[0]  # a live commander is accepted
    assert not m.ok(3.2)[0]  # and the stop follows within cmd_max_age


def test_motion_zero_setpoint_coast_window():
    m = mc(coast_max_s=1.0)
    m.on_cmd(10.0, 0.2)
    quiet(m, 10.0, 10.1, lin=0.2)
    m.on_cmd(10.1, 0.0)  # setpoint becomes zero while the robot still moves; commands are silent from now on
    quiet(m, 10.1, 10.3, lin=0.2)
    assert m.ok(10.3)[0]  # 0.2 s after the zero: inside the coast window (command also still fresh until 10.25)
    quiet(m, 10.3, 11.0, lin=0.2)
    assert m.ok(11.0)[0]  # 0.9 s after the zero
    quiet(m, 11.0, 11.2, lin=0.2)
    assert not m.ok(11.2)[0]  # window expired (1.1 s) and still moving
    quiet(m, 11.2, 11.3)  # finally quiet
    assert not m.ok(11.2)[0] and not m.ok(11.25)[0] and m.ok(11.3)[0]  # settle_s again
    # the coast window applies to a zero setpoint only
    m.on_cmd(11.3, 0.1)
    quiet(m, 11.3, 11.8, lin=0.1)
    assert not m.ok(11.8)[0]


def test_motion_zero_setpoint_stationary_after_commands_stay_healthy():
    m = mc()
    m.on_cmd(1.0, 0.0)
    quiet(m, 1.0, 20.0)
    assert m.ok(20.0)[0]  # idle for a long time


def test_motion_zero_command_streams_do_not_extend_the_coast_window():
    m = mc(coast_max_s=1.0)
    m.on_cmd(1.0, 0.0)
    quiet(m, 1.0, 2.5, lin=0.2)
    for t in (1.2, 1.4, 1.6):
        m.on_cmd(t, 0.0)  # repeated zeros, the setpoint "became zero" at 1.0
    assert not m.ok(2.5)[0]


def test_motion_thresholds():
    m = mc(linear_threshold=0.1, angular_threshold=0.5)
    quiet(m, 1.0, 1.2, lin=0.09, ang=0.4)
    assert m.ok(1.2)[0]
    m = mc(linear_threshold=0.1, angular_threshold=0.5)
    quiet(m, 1.0, 1.2, lin=0.11)
    assert not m.ok(1.2)[0]
    m = mc(linear_threshold=0.1, angular_threshold=0.5)
    quiet(m, 1.0, 1.2, ang=-0.6)
    assert not m.ok(1.2)[0]  # sign does not matter


def test_motion_odom_stale_or_missing_is_not_ok():
    m = mc(odom_max_age_s=0.3)
    assert not m.ok(1.0)[0]
    quiet(m, 0.8, 1.0)
    assert m.ok(1.3)[0]
    ok, detail = m.ok(1.31)
    assert not ok and "stale" in detail
    m.on_cmd(1.3, 0.2)
    assert not m.ok(1.31)[0]  # a command cannot make up for missing odometry


def test_motion_non_finite_values_are_not_ok():
    for bad in (float("nan"), float("inf"), float("-inf")):
        m = mc()
        m.on_cmd(1.0, 0.2)
        m.on_odom(1.0, bad, 0.0)
        assert not m.ok(1.0)[0]
        m.on_odom(1.0, 0.0, bad)
        assert not m.ok(1.0)[0]
        # a non-finite command is a non-zero setpoint and not a fresh command
        m = mc()
        quiet(m, 0.0, 1.0)
        assert m.ok(1.0)[0]
        m.on_cmd(1.0, bad, 0.0)
        assert not m.ok(1.0)[0]
        m.on_cmd(1.0, 0.0, bad)
        assert not m.ok(1.01)[0]
    m = mc()
    quiet(m, 0.0, 1.0)
    assert not m.ok(float("nan"))[0]


def test_motion_rejects_bad_parameters():
    for kw in ({"linear_threshold": 0}, {"cmd_max_age_s": -1}, {"odom_max_age_s": float("nan")}, {"settle_s": 0}, {"coast_max_s": float("inf")}):
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


def test_motion_stream_of_zero_commands_gives_no_credit_while_moving():
    m = mc(coast_max_s=1.0)
    m.on_cmd(10.0, 0.2)
    t = 10.0
    while t < 12.0:  # zeros every 50 ms while odometry keeps reporting motion
        t = round(t + 0.05, 3)
        m.on_odom(t, 0.2, 0.0)
        m.on_cmd(t, 0.0, 0.0)
        if t > 10.2 and t < 11.0:
            assert m.ok(t)[0], t  # inside the coast window (started at 10.05)
    m.on_odom(12.0, 0.2, 0.0)
    m.on_cmd(12.0, 0.0, 0.0)
    assert not m.ok(12.0)[0]  # fresh zero commands do not keep it alive past coast_max_s


def test_motion_repeated_zeros_do_not_restart_the_coast_window():
    m = mc(coast_max_s=1.0)
    m.on_cmd(1.0, 0.0)
    for t in (1.3, 1.6, 1.9, 2.2):
        m.on_cmd(t, 0.0)
    m.on_odom(2.2, 0.2, 0.0)
    assert not m.ok(2.2)[0]  # window started at 1.0, expired at 2.0
    m.on_cmd(2.3, 0.1)  # a non-zero command clears the window ...
    m.on_cmd(2.4, 0.0)  # ... and the next zero starts a new one
    m.on_odom(2.5, 0.2, 0.0)
    assert m.ok(2.5)[0]
    m.on_odom(3.5, 0.2, 0.0)
    assert not m.ok(3.5)[0]


def test_motion_queue_stale_command_updates_the_setpoint_without_credit():
    m = mc()
    quiet(m, 0.0, 1.0)
    assert m.ok(1.0)[0]
    m.on_cmd(1.0, 0.2, 0.0, fresh=False)  # waited in the queue: the stand-in forwarded it, the setpoint is non-zero
    ok, detail = m.ok(1.0)
    assert not ok and "non-zero setpoint" in detail
    m.on_cmd(1.0, 0.2, 0.0)
    assert m.ok(1.0)[0]  # a fresh one gives credit
    m = mc()
    m.on_cmd(1.0, 0.2)
    m.on_cmd(1.5, 0.0, 0.0, fresh=False)  # a stale zero makes the held setpoint zero
    quiet(m, 1.5, 1.7)
    assert m.ok(1.7)[0]


# ---------------------------------------------------------------- M8: CommandChainGuard (rule A)

def chain(**kw):
    return CommandChainGuard(**{"in_max_age_s": 0.35, "stop_grace_s": 1.0, **kw})


def test_chain_nothing_replayed_is_ok():
    g = chain()
    assert g.ok(1.0)[0]                  # no output ever seen
    g.on_out(1.0, 0.0, 0.0)
    assert g.ok(1.1)[0]                  # zero output, no input


def test_chain_output_without_any_input_is_not_ok():
    g = chain()
    g.on_out(1.0, 0.2, 0.0)
    ok, why = g.ok(1.0)
    assert not ok and "no input ever seen" in why


def test_chain_live_commander_keeps_it_ok_and_a_silent_one_trips_at_the_bound():
    g = chain(in_max_age_s=0.35)
    t = 10.0
    for i in range(10):                  # 5 Hz input, 20 Hz output
        g.on_in(t, 0.2, 0.0)
        for k in range(4):
            g.on_out(t + k * 0.05, 0.2, 0.0)
            assert g.ok(t + k * 0.05)[0]
        t += 0.2
    last_in = t - 0.2
    assert g.ok(last_in + 0.35)[0]
    ok, why = g.ok(last_in + 0.36)
    assert not ok and "no non-zero input newer than 0.35" in why
    g.on_in(last_in + 0.4, 0.2, 0.0)     # the commander comes back
    assert g.ok(last_in + 0.4)[0]


def test_chain_commanded_stop_gets_a_bounded_grace():
    g = chain(stop_grace_s=1.0)
    g.on_in(5.0, 0.2, 0.0)
    g.on_out(5.0, 0.2, 0.0)
    g.on_in(5.2, 0.0, 0.0)               # commander says stop; the smoother decelerates
    assert g.ok(5.9)[0]
    assert g.ok(6.19)[0]
    ok, why = g.ok(6.21)
    assert not ok and "after the input went to zero" in why


def test_chain_repeated_zero_inputs_do_not_extend_the_grace_and_nonzero_resets_it():
    g = chain(stop_grace_s=1.0)
    g.on_in(5.0, 0.2, 0.0)
    g.on_out(5.0, 0.2, 0.0)
    for i in range(30):
        g.on_in(5.2 + i * 0.05, 0.0, 0.0)
    assert not g.ok(6.8)[0]
    g.on_in(7.0, 0.0, 0.1)               # non-zero again (turn)
    assert g.ok(7.3)[0]
    assert not g.ok(7.4)[0]


def test_chain_output_back_to_zero_is_ok_again():
    g = chain()
    g.on_in(1.0, 0.2, 0.0)
    g.on_out(1.0, 0.2, 0.0)
    assert not g.ok(3.0)[0]
    g.on_out(3.0, 0.0, 0.0)
    assert g.ok(3.0)[0]


def test_chain_non_finite_and_backwards_time_are_not_ok():
    g = chain()
    g.on_in(1.0, 0.2, 0.0)
    g.on_out(1.0, 0.2, 0.0)
    assert g.ok(1.1)[0]
    assert not g.ok(float("nan"))[0]
    assert not g.ok(0.5)[0]              # clock jumped back
    g.on_in(1.1, float("nan"), 0.0)
    ok, why = g.ok(1.1)
    assert not ok and "non-finite" in why
    g.on_in(1.2, 0.2, 0.0)
    assert g.ok(1.2)[0]
    g.on_out(1.2, float("inf"), 0.0)     # non-finite output = a non-zero setpoint
    g.on_out(float("nan"), 0.0, 0.0)
    assert g.ok(1.3)[0]                  # still covered by the fresh input
    assert not g.ok(2.0)[0]


def test_chain_output_zero_ignored_input_values_below_epsilon_count_as_zero():
    g = chain()
    g.on_in(1.0, 1e-12, 0.0)
    g.on_out(1.0, 0.2, 0.0)
    assert g.ok(1.5)[0]                  # inside the stop grace (input zero since 1.0)
    assert not g.ok(2.1)[0]


@pytest.mark.parametrize("kw", [{"in_max_age_s": 0}, {"in_max_age_s": float("nan")}, {"stop_grace_s": -1}, {"stop_grace_s": float("inf")}])
def test_chain_rejects_bad_parameters(kw):
    with pytest.raises(ValueError):
        chain(**kw)


# ---------------------------------------------------------------- M8: ReverseFixGuard (rule B)

def rev(**kw):
    return ReverseFixGuard(**{"fix_max_age_s": 1.5, **kw})


def test_reverse_not_reversing_is_ok_without_fixes():
    g = rev()
    assert g.ok(1.0)[0]                  # no command ever seen
    g.on_cmd(1.0, 0.1)
    assert g.ok(100.0)[0]                # driving forward: the rule does not apply
    g.on_cmd(100.0, 0.0)
    assert g.ok(200.0)[0]


def test_reverse_needs_a_fresh_fix_and_trips_at_the_bound():
    g = rev(fix_max_age_s=1.5)
    g.on_fix(10.0)
    g.on_cmd(10.1, -0.06)
    assert g.ok(11.5)[0]
    ok, why = g.ok(11.65)
    assert not ok and "dock fix" in why
    g.on_fix(11.7)
    assert g.ok(13.1)[0]
    assert not g.ok(13.3)[0]


def test_reverse_start_reference_gives_an_old_fix_one_bound_to_refresh():
    g = rev(fix_max_age_s=1.0, from_reverse_start=True)
    g.on_fix(0.0)
    g.on_cmd(20.0, -0.06)
    assert g.ok(20.9)[0]
    assert not g.ok(21.1)[0]
    s = rev(fix_max_age_s=1.0, from_reverse_start=False)
    s.on_fix(0.0)
    s.on_cmd(20.0, -0.06)
    assert not s.ok(20.0)[0]             # strict: the last fix must be fresh from the first reversing command


def test_reverse_no_fix_ever_trips_strict_mode_and_times_out_in_start_mode():
    s = rev(from_reverse_start=False)
    s.on_cmd(1.0, -0.06)
    ok, why = s.ok(1.0)
    assert not ok and "no dock fix ever seen" in why
    g = rev(fix_max_age_s=1.0)
    g.on_cmd(1.0, -0.06)
    assert g.ok(1.5)[0] and not g.ok(2.1)[0]


def test_reverse_threshold_and_stop_clear_the_rule():
    g = rev(reverse_threshold=0.02)
    g.on_fix(0.0)
    g.on_cmd(5.0, -0.019)                # creep below the threshold is not reversing
    assert g.ok(9.0)[0]
    g.on_cmd(9.0, -0.02)
    assert g.ok(9.5)[0] and not g.ok(10.6)[0]
    g.on_cmd(10.7, 0.0)
    assert g.ok(10.7)[0]


def test_reverse_non_finite_command_counts_as_reversing_and_backwards_time_is_not_ok():
    g = rev()
    g.on_fix(0.0)
    g.on_cmd(0.1, float("nan"))
    assert g.ok(0.2)[0] and not g.ok(5.0)[0]
    assert not g.ok(float("nan"))[0]
    g.on_cmd(5.0, 0.0)
    assert g.ok(5.0)[0]
    assert not g.ok(1.0)[0]


def test_reverse_fix_with_non_finite_time_is_ignored():
    g = rev(from_reverse_start=False)
    g.on_cmd(1.0, -0.06)
    g.on_fix(float("nan"))
    assert not g.ok(1.0)[0]


@pytest.mark.parametrize("kw", [{"fix_max_age_s": 0}, {"fix_max_age_s": float("nan")}, {"reverse_threshold": -0.1}])
def test_reverse_rejects_bad_parameters(kw):
    with pytest.raises(ValueError):
        rev(**kw)


# ---------------------------------------------------------------- M8: parsing

def test_parse_command_chain_and_reverse_fix_defaults():
    c, r = parse_watches(doc(
        {"component_id": "command_chain", "command_chain": {"in_topic": "/cmd_vel_nav", "out_topic": "/cmd_vel_smoothed"}},
        {"component_id": "dock_fix", "reverse_fix": {"cmd_topic": "/cmd_vel", "fix_topic": "/f", "fix_type": "geometry_msgs/msg/PoseStamped"}},
    ))
    assert c.is_chain and not c.is_motion and c.chain["in_max_age_s"] == 0.35 and c.chain["stop_grace_s"] == 1.0
    assert c.topic == "/odom" and c.msg_type == "nav_msgs/msg/Odometry"
    assert r.is_reverse_fix and r.reverse_fix["fix_max_age_s"] == 1.5 and r.reverse_fix["from_reverse_start"] is True


@pytest.mark.parametrize(
    "bad",
    [
        doc({"component_id": "a", "command_chain": {"in_topic": "/i"}}),
        doc({"component_id": "a", "command_chain": {"in_topic": "/i", "out_topic": "/o", "in_max_age_s": 0}}),
        doc({"component_id": "a", "command_chain": {"in_topic": "/i", "out_topic": "/o", "stop_grace_s": "x"}}),
        doc({"component_id": "a", "topic": "t", "command_chain": {"in_topic": "/i", "out_topic": "/o"}}),
        doc({"component_id": "a", "reverse_fix": {"cmd_topic": "/c", "fix_topic": "/f"}}),
        doc({"component_id": "a", "reverse_fix": {"cmd_topic": "/c", "fix_topic": "/f", "fix_type": "x/msg/Y", "fix_max_age_s": -1}}),
        doc({"component_id": "a", "reverse_fix": {"cmd_topic": "/c", "fix_topic": "/f", "fix_type": "x/msg/Y"},
             "command_chain": {"in_topic": "/i", "out_topic": "/o"}}),
        doc({"component_id": "a", "tf": {"parent": "m", "child": "o"}, "reverse_fix": {"cmd_topic": "/c", "fix_topic": "/f", "fix_type": "x/msg/Y"}}),
        doc({"component_id": "a", "command_chain": "oops"}),
    ],
)
def test_parse_rejects_bad_m8_watches(bad):
    with pytest.raises(ValueError):
        parse_watches(bad)


# ---------------------------------------------------------------- M8 review fixes: rules that must not switch off silently

def test_chain_graph_problem_is_not_ok_and_recovers():
    g = chain()
    assert g.ok(1.0)[0]
    g.set_graph(False, "/cmd_vel_nav: no publisher")
    ok, why = g.ok(1.0)
    assert not ok and "no publisher" in why
    g.set_graph(True)
    assert g.ok(1.0)[0]


def test_chain_motion_without_any_output_is_not_ok_after_the_grace():
    g = chain(motion_grace_s=0.5)
    g.on_odom(1.0, 0.0, 0.0)
    assert g.ok(1.0)[0]
    g.on_odom(2.0, 0.2, 0.0)              # odometry shows motion, the stage never emitted anything
    assert g.ok(2.4)[0]                   # inside the grace (a command may be on its way)
    ok, why = g.ok(2.6)
    assert not ok and "no output was ever seen" in why
    g.on_odom(2.7, 0.0, 0.0)              # standing again: healthy
    assert g.ok(2.7)[0]
    g.on_odom(3.0, 0.0, 0.3)              # a turn counts as motion
    assert not g.ok(3.6)[0]
    g.on_out(3.6, 0.0, 0.3)               # the stage has emitted: the old rules apply (input missing -> not ok, zero output -> ok)
    assert "no input ever seen" in g.ok(3.6)[1]
    g.on_out(3.7, 0.0, 0.0)
    assert g.ok(3.8)[0]


def test_chain_motion_check_needs_odometry_and_treats_non_finite_odometry_as_motion():
    g = chain()
    assert g.ok(100.0)[0]                 # no on_odom calls: the check does nothing (documented)
    g.on_odom(float("nan"), 0.0, 0.0)
    assert not g.ok(1.0)[0]
    h = chain()
    h.on_odom(1.0, float("nan"), 0.0)
    assert not h.ok(2.0)[0]


def test_reverse_by_any_writer_trips_without_a_fix():
    """Documented precondition of rule B: a Nav2 BackUp or teleop that reverses is 'reversing' too; no dock fix exists, so it trips."""
    g = rev(fix_max_age_s=1.85, from_reverse_start=False)
    g.on_cmd(10.0, -0.12)                 # e.g. BackUp at -0.12 m/s, no detector in the stack
    assert not g.ok(10.0)[0]
    h = rev(fix_max_age_s=1.85, from_reverse_start=True)
    h.on_cmd(10.0, -0.12)
    assert h.ok(11.8)[0] and not h.ok(12.0)[0]


def test_reverse_graph_problem_is_not_ok():
    g = rev()
    g.set_graph(False, "/cmd_vel: publisher type ['geometry_msgs/msg/TwistStamped'] is not geometry_msgs/msg/Twist")
    ok, why = g.ok(1.0)
    assert not ok and "TwistStamped" in why
    g.set_graph(True)
    assert g.ok(1.0)[0]


def test_parse_message_types_and_motion_options():
    (c,) = parse_watches(doc({"component_id": "c", "command_chain": {"in_topic": "/i", "out_topic": "/o", "msg_type": "geometry_msgs/msg/TwistStamped"}}))
    assert c.chain["msg_type"] == "geometry_msgs/msg/TwistStamped" and c.chain["motion_grace_s"] == 0.5
    (r,) = parse_watches(doc({"component_id": "r", "reverse_fix": {"cmd_topic": "/c", "fix_topic": "/f", "fix_type": "x/msg/Y"}}))
    assert r.reverse_fix["cmd_type"] == "geometry_msgs/msg/Twist"
    for bad in (
        doc({"component_id": "c", "command_chain": {"in_topic": "/i", "out_topic": "/o", "msg_type": "std_msgs/msg/String"}}),
        doc({"component_id": "c", "command_chain": {"in_topic": "/i", "out_topic": "/o", "motion_grace_s": 0}}),
        doc({"component_id": "r", "reverse_fix": {"cmd_topic": "/c", "fix_topic": "/f", "fix_type": "x/msg/Y", "cmd_type": "nope"}}),
    ):
        with pytest.raises(ValueError):
            parse_watches(bad)
