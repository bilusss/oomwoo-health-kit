import signal

import ohk_harness as h


def generate_test_description():
    return h.build_description()


class TestStallWorkPath(h.OhkCase):
    def test_stalled_work_loop_stops_robot(self):
        """Work loop blocks (process + executor alive); work-path beats stop -> robot stops."""
        self.wait_run()
        t0, lat = self.signal_and_time_stop("critical_a", signal.SIGUSR1)
        self.record("h_stall_work", latency_ms=lat * 1000)
        self.assertLess(lat, 0.45)
        self.assert_blames("critical_a", t0)
