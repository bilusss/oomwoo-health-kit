"""M8 control for rule A: with the M7 watch set (no command_chain) a dead commander behind the smoother imitation is NOT detected: the
stand-in stays in RUN while the smoother re-emits the held setpoint, and the first zero at the gate is the smoother's own ramp after its
1.0 s velocity_timeout. This documents the gap the rule closes."""

import os
import signal
import time

import ohk_nav_harness as n


def generate_test_description():
    return n.build_nav_description(fakes=("scan", "odom", "tf", "polygon", "smoother", "commander"))


class TestChainInvisibleWithoutRule(n.NavCase):
    PROBE_FEED = False

    def test_dead_commander_is_invisible_to_the_m7_watch_set(self):
        self.wait_quiet_run()
        self.set_enable(True)
        self.wait_run(10.0)
        time.sleep(1.5)
        t0 = time.monotonic()
        os.kill(self.pid("commander"), signal.SIGKILL)
        self.wait_for(lambda: self.first_zero_since(t0) is not None, 3.0, "the smoother's own ramp to zero")
        lat = self.first_zero_since(t0)
        self.record("u4_chain_invisible_without_rule", first_zero_ms=lat * 1000)
        self.assertGreater(lat, 1.0, "motion continued for the smoother's velocity_timeout")
        self.assertTrue(all(s["state"] == "run" for _, s in self.states_since(t0)), "the stand-in never left RUN")
