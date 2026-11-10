import signal
import time

import ohk_harness as h


def generate_test_description():
    return h.build_description()


class TestAggregator(h.OhkCase):
    def test_aggregator_killed(self):
        self.wait_run()
        t0, lat = self.signal_and_time_stop("health_monitor", signal.SIGKILL)
        self.record("f_aggregator", latency_ms=lat * 1000)
        self.assertLess(lat, 0.25)
        self.wait_for(
            lambda: any(s["state"] == "cpu_reset" for _, s in self.states_since(t0)), 4.0, "CPU_RESET"
        )
        t_reset = next(t for t, s in self.states_since(t0) if s["state"] == "cpu_reset")
        self.record("f_cpu_reset", reset_ms=(t_reset - t0) * 1000)
        self.assertTrue(all(not nz for _, nz in self.cmds_since(t0 + lat)))
