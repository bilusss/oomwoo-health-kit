import signal

import ohk_harness as h
import ohk_nav_harness as n

BOUND_S = n.SCAN_MAX_AGE + h.SOFT_STOP_S + h.TICK_S + h.MARGIN_S


def generate_test_description():
    return n.build_nav_description()


class TestScanStop(n.NavCase):
    def test_scan_publisher_frozen(self):
        self.wait_idle_run()
        t0, lat = self.signal_and_time_stop("scan", signal.SIGSTOP, timeout=3.0)
        self.record("m_scan_sigstop", latency_ms=lat * 1000, bound_ms=BOUND_S * 1000)
        self.assertLess(lat, BOUND_S)
        self.assert_blames("lidar", t0)
        self.wait_for(lambda: self.current_state() != "run", 1.0, "state leaves RUN")
        self.assertTrue(all(not nz for _, nz in self.cmds_since(t0 + lat)))
