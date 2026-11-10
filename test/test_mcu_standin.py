import math

import pytest

from oomwoo_health_kit.mcu_standin import McuStandin, McuState, Velocity, ZERO_VELOCITY

CMD = Velocity(0.3, 0.5)


def armed(**kw) -> McuStandin:
    mcu = McuStandin(**kw)
    for t in (0.0, 0.1, 0.2):
        mcu.heartbeat(t)
    assert mcu.state is McuState.RUN
    return mcu


def test_defaults_and_initial_state():
    mcu = McuStandin()
    assert (mcu.soft_stop_timeout_s, mcu.cpu_reset_timeout_s, mcu.rearm_count) == (0.2, 2.0, 3)
    assert mcu.state is McuState.UNARMED
    assert mcu.transitions == []
    assert (mcu.heartbeats_received, mcu.heartbeats_ignored, mcu.soft_stops, mcu.resets) == (0, 0, 0, 0)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"soft_stop_timeout_s": 0.0},
        {"soft_stop_timeout_s": -1.0},
        {"soft_stop_timeout_s": 2.0},
        {"soft_stop_timeout_s": 3.0},
        {"rearm_count": 0},
    ],
)
def test_parameter_validation(kwargs):
    with pytest.raises(ValueError):
        McuStandin(**kwargs)


def test_unarmed_stays_unarmed_without_heartbeat():
    mcu = McuStandin()
    assert mcu.update(100.0).state is McuState.UNARMED
    assert mcu.gate(100.0, CMD) == ZERO_VELOCITY


def test_arms_after_rearm_count_heartbeats():
    mcu = McuStandin()
    mcu.heartbeat(0.0)
    mcu.heartbeat(0.1)
    assert mcu.state is McuState.UNARMED
    assert mcu.gate(0.1, CMD) == ZERO_VELOCITY
    mcu.heartbeat(0.2)
    assert mcu.state is McuState.RUN
    assert mcu.gate(0.2, CMD) == CMD
    t = mcu.transitions[-1]
    assert (t.time, t.from_state, t.to_state) == (0.2, McuState.UNARMED, McuState.RUN)
    assert t.reason


def test_rearm_count_one_arms_on_first_heartbeat():
    mcu = McuStandin(rearm_count=1)
    mcu.heartbeat(5.0)
    assert mcu.state is McuState.RUN


def test_soft_stop_boundary():
    mcu = armed()
    assert mcu.update(0.2 + 0.2).state is McuState.RUN
    assert mcu.update(0.2 + 0.2001).state is McuState.SOFT_STOP
    assert mcu.soft_stops == 1
    assert mcu.gate(0.5, CMD) == ZERO_VELOCITY


def test_cpu_reset_boundary_and_reported_once():
    mcu = armed()
    assert mcu.update(0.2 + 2.0).state is McuState.SOFT_STOP
    status = mcu.update(0.2 + 2.0001)
    assert status.state is McuState.CPU_RESET and status.reset_asserted_now
    assert mcu.reset_asserted
    again = mcu.update(5.0)
    assert again.state is McuState.CPU_RESET and not again.reset_asserted_now
    assert mcu.resets == 1


def test_run_straight_to_reset_on_long_gap():
    mcu = armed()
    assert mcu.update(10.0).state is McuState.CPU_RESET
    assert mcu.transitions[-1].from_state is McuState.RUN
    assert (mcu.soft_stops, mcu.resets) == (0, 1)


def test_soft_stop_recovers_after_streak():
    mcu = armed()
    mcu.update(1.0)
    mcu.heartbeat(1.0)
    mcu.heartbeat(1.1)
    assert mcu.state is McuState.SOFT_STOP
    mcu.heartbeat(1.2)
    assert mcu.state is McuState.RUN
    assert mcu.gate(1.2, CMD) == CMD


def test_cpu_reset_recovers_after_streak():
    mcu = armed()
    mcu.update(10.0)
    for t in (10.0, 10.1, 10.2):
        mcu.heartbeat(t)
    assert mcu.state is McuState.RUN
    assert not mcu.reset_asserted


def test_stray_heartbeat_does_not_rearm():
    mcu = armed()
    mcu.update(10.0)
    mcu.heartbeat(10.0)
    assert mcu.state is McuState.CPU_RESET
    assert mcu.gate(10.0, CMD) == ZERO_VELOCITY


def test_broken_streak_restarts():
    mcu = armed()
    mcu.update(1.0)
    mcu.heartbeat(1.0)
    mcu.heartbeat(1.1)
    mcu.heartbeat(1.5)
    mcu.heartbeat(1.6)
    assert mcu.state is McuState.SOFT_STOP
    mcu.heartbeat(1.7)
    assert mcu.state is McuState.RUN


def test_gap_exactly_soft_timeout_keeps_streak():
    mcu = McuStandin()
    for t in (0.0, 0.2, 0.4):
        mcu.heartbeat(t)
    assert mcu.state is McuState.RUN


def test_late_heartbeat_in_run_drops_to_soft_stop():
    mcu = armed()
    mcu.heartbeat(1.0)
    assert mcu.state is McuState.SOFT_STOP
    assert mcu.soft_stops == 1


def test_out_of_order_heartbeat_ignored_and_counted():
    mcu = armed()
    received = mcu.heartbeats_received
    mcu.heartbeat(0.05)
    assert mcu.heartbeats_ignored == 1
    assert mcu.heartbeats_received == received
    assert mcu.state is McuState.RUN
    assert mcu.update(0.4).state is McuState.RUN


def test_out_of_order_does_not_advance_streak():
    mcu = McuStandin()
    mcu.heartbeat(0.0)
    mcu.heartbeat(0.1)
    mcu.heartbeat(0.05)
    assert mcu.state is McuState.UNARMED
    mcu.heartbeat(0.2)
    assert mcu.state is McuState.RUN


def test_counters_and_transition_log():
    mcu = armed()
    mcu.update(1.0)
    mcu.update(10.0)
    assert mcu.heartbeats_received == 3
    assert [(t.from_state, t.to_state) for t in mcu.transitions] == [
        (McuState.UNARMED, McuState.RUN),
        (McuState.RUN, McuState.SOFT_STOP),
        (McuState.SOFT_STOP, McuState.CPU_RESET),
    ]
    assert [t.time for t in mcu.transitions] == [0.2, 1.0, 10.0]
    assert (mcu.soft_stops, mcu.resets) == (1, 1)


@pytest.mark.parametrize("state_setup", ["unarmed", "soft_stop", "cpu_reset"])
def test_gate_zero_in_every_non_run_state(state_setup):
    mcu = McuStandin()
    if state_setup != "unarmed":
        mcu = armed()
        mcu.update(1.0 if state_setup == "soft_stop" else 10.0)
    assert mcu.gate(10.0 if state_setup == "cpu_reset" else 1.0, CMD) == ZERO_VELOCITY


def test_gate_passes_command_unchanged_in_run():
    mcu = armed()
    assert mcu.gate(0.3, Velocity(-0.1, 0.0)) == Velocity(-0.1, 0.0)


def test_gate_applies_timeouts():
    mcu = armed()
    assert mcu.gate(0.5, CMD) == ZERO_VELOCITY
    assert mcu.state is McuState.SOFT_STOP


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_non_finite_time_is_counted_and_changes_nothing(bad):
    mcu = armed()
    log = list(mcu.transitions)
    assert mcu.heartbeat(bad).state is McuState.RUN
    assert mcu.update(bad).state is McuState.RUN
    assert mcu.gate(bad, CMD) == ZERO_VELOCITY
    assert mcu.invalid_time == 3
    assert mcu.transitions == log
    assert mcu.heartbeats_received == 3
    assert mcu.gate(0.3, CMD) == CMD
    assert mcu.update(10.0).state is McuState.CPU_RESET


def test_nan_heartbeat_does_not_poison_gate_when_stale():
    mcu = armed()
    mcu.heartbeat(math.nan)
    assert mcu.gate(5.0, CMD) == ZERO_VELOCITY


def test_non_finite_gives_no_streak_credit():
    mcu = McuStandin()
    mcu.heartbeat(0.0)
    mcu.heartbeat(0.1)
    mcu.heartbeat(math.nan)
    assert mcu.state is McuState.UNARMED


def test_gate_with_older_time_than_latest_seen_is_zero():
    mcu = armed()
    assert mcu.gate(-5.0, CMD) == ZERO_VELOCITY
    assert mcu.state is McuState.RUN


def test_gate_older_than_last_update_in_run_is_zero():
    mcu = armed()
    mcu.update(0.35)
    assert mcu.gate(0.25, CMD) == ZERO_VELOCITY
    assert mcu.gate(0.35, CMD) == CMD


def test_stale_heartbeat_is_ignored_after_update():
    mcu = armed()
    mcu.update(10.0)
    mcu.heartbeat(0.3)
    assert mcu.heartbeats_ignored == 1
    assert mcu.state is McuState.CPU_RESET


def test_update_with_older_time_uses_latest_seen():
    mcu = armed()
    mcu.update(10.0)
    assert mcu.update(0.3).state is McuState.CPU_RESET


def test_streak_reset_on_cpu_reset_requires_full_rearm():
    mcu = armed()
    mcu.update(10.0)
    mcu.heartbeat(10.0)
    mcu.heartbeat(10.1)
    assert mcu.state is McuState.CPU_RESET
    mcu.heartbeat(10.2)
    assert mcu.state is McuState.RUN


def test_streak_reset_on_soft_stop_requires_full_rearm():
    mcu = armed()
    mcu.update(0.5)
    assert mcu.state is McuState.SOFT_STOP
    mcu.heartbeat(0.5)
    mcu.heartbeat(0.6)
    assert mcu.state is McuState.SOFT_STOP
    mcu.heartbeat(0.7)
    assert mcu.state is McuState.RUN


def test_far_future_heartbeat_then_normal_time_stays_inhibited():
    mcu = armed()
    mcu.heartbeat(1e9)
    for t in (1.0, 1.1, 1.2):
        mcu.heartbeat(t)
    assert mcu.heartbeats_ignored == 3
    assert mcu.state is McuState.CPU_RESET
    assert mcu.gate(1.3, CMD) == ZERO_VELOCITY


def test_heartbeat_reports_reset_when_it_triggers_it():
    mcu = armed()
    status = mcu.heartbeat(10.0)
    assert status.state is McuState.CPU_RESET
    assert status.reset_asserted_now
    assert mcu.reset_asserted
    assert not mcu.heartbeat(10.1).reset_asserted_now


def test_reset_asserted_visible_via_gate_only():
    mcu = armed()
    assert not mcu.reset_asserted
    mcu.gate(10.0, CMD)
    assert mcu.reset_asserted
