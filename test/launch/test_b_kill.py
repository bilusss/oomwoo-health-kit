import signal

import ohk_harness as h


def generate_test_description():
    return h.build_description()


class TestKill(h.OhkCase):
    def test_kill_critical(self):
        self.wait_run()
        t0, lat = self.signal_and_time_stop("critical_a", signal.SIGKILL)
        self.record("b_kill", latency_ms=lat * 1000)
        self.assertLess(lat, h.FAULT_TO_STOP_MAX_S)
        self.assert_blames("critical_a", t0)
        self.wait_for(lambda: self.current_state() != "run", 1.0, "state leaves RUN")
