"""M8 rule A, fault case: the commander upstream of the velocity smoother dies (SIGKILL). The smoother imitation keeps re-emitting the
held setpoint on its output and on the stand-in input, so drive_supervision stays healthy (test u4 shows that without the rule); the
command_chain component must stop the robot within its input bound plus the usual aggregator and stand-in latency."""

import signal
import time

import ohk_harness as h
import ohk_nav_harness as n

IN_MAX_AGE_S = n.CHAIN_IN_MAX_AGE_S      # from config/nav_watch_amcl_chain.yaml
COMMANDER_PERIOD_S = 0.2                  # the fake commander runs at 5 Hz like controller_server
# worst case: the last input was just before the signal (the bound counts from it) + component max_age + soft stop + tick + margin
BOUND_S = IN_MAX_AGE_S + 0.15 + 0.04 + h.SOFT_STOP_S + h.TICK_S + h.MARGIN_S


def generate_test_description():
    return n.m8_description("chain", fakes=("scan", "odom", "tf", "polygon", "smoother", "commander"))


class TestChainDeadCommander(n.NavCase):
    PROBE_FEED = False

    def test_dead_commander_behind_the_smoother_is_detected(self):
        self.wait_quiet_run()
        self.set_enable(True)
        self.wait_run(10.0)
        self.wait_stack(lambda s: s["state"].startswith("healthy"), 3.0, "healthy while the commander is alive")
        time.sleep(1.5)
        self.assertEqual(self.current_state(), "run", "no soft stop while the commander is alive")
        t0, lat = self.signal_and_time_stop("commander", signal.SIGKILL, timeout=BOUND_S + 1.0)
        self.record("u1_chain_dead_commander", latency_ms=lat * 1000, bound_ms=BOUND_S * 1000)
        self.assertLess(lat, BOUND_S)
        # lower bound: the rule cannot trip before in_max_age after the last input (at most one commander period before the signal)
        self.assertGreater(lat, IN_MAX_AGE_S - COMMANDER_PERIOD_S, "not before the input bound could have expired")
        self.assertEqual([s for _, s in self.stacks_since(t0 - 1.5) if s["state"] == "fault" and _ < t0], [], "no fault before the signal")
        self.assert_blames("command_chain", t0)
        self.assertTrue(any(s["state"] != "run" for _, s in self.states_since(t0)), "the stand-in left RUN")
