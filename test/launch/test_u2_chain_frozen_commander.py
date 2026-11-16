"""M8 rule A, fault case: the commander freezes (SIGSTOP) instead of dying; same expectation as u1, and it must resume cleanly."""

import os
import signal
import time

import ohk_harness as h
import ohk_nav_harness as n

BOUND_S = n.CHAIN_IN_MAX_AGE_S + 0.15 + 0.04 + h.SOFT_STOP_S + h.TICK_S + h.MARGIN_S


def generate_test_description():
    return n.m8_description("chain", fakes=("scan", "odom", "tf", "polygon", "smoother", "commander"))


class TestChainFrozenCommander(n.NavCase):
    PROBE_FEED = False

    def test_frozen_commander_behind_the_smoother_is_detected_and_recovers(self):
        self.wait_quiet_run()
        self.set_enable(True)
        self.wait_run(10.0)
        self.wait_stack(lambda s: s["state"].startswith("healthy"), 3.0, "healthy while the commander is alive")
        time.sleep(1.5)
        t0, lat = self.signal_and_time_stop("commander", signal.SIGSTOP, timeout=BOUND_S + 1.0)
        self.record("u2_chain_frozen_commander", latency_ms=lat * 1000, bound_ms=BOUND_S * 1000)
        self.assertLess(lat, BOUND_S)
        self.assertGreater(lat, n.CHAIN_IN_MAX_AGE_S - 0.2, "not before the input bound could have expired")
        self.assert_blames("command_chain", t0)
        os.kill(self.pid("commander"), signal.SIGCONT)
        self.wait_run(30.0)   # arming restarts; the commander is alive again
