import os
import signal
import time

import ohk_harness as h
import ohk_nav_harness as n

# Signal -> next scan message (<= one 0.2 s period; the stale stamp is only visible on a new message) -> aggregator
# withholds the MCU heartbeat on its next 50 ms tick -> stand-in soft-stops 0.15 s after the last heartbeat.
SCAN_PERIOD_S = 0.2
BOUND_S = SCAN_PERIOD_S + 0.05 + h.SOFT_STOP_S + h.TICK_S + h.MARGIN_S


def generate_test_description():
    return n.build_nav_description()


class TestStaleStamp(n.NavCase):
    def test_stale_header_stamp_is_degraded_and_unhealthy(self):
        self.wait_idle_run()
        t0, lat = self.signal_and_time_stop("scan", signal.SIGUSR1)  # stamps now 2 s old
        with self.probe.lock:
            details = [d for ts, c, d in self.probe.comp_details if c == "lidar" and ts >= t0 and d]
        self.assertTrue(details and "stamp age" in details[0], details)
        # core.py: only ok/healthy count as well, so degraded on a critical component is "unhealthy", not "stale".
        self.wait_stack(
            lambda s: s["state"] == "fault" and s["unhealthy_critical"] == ["lidar"] and s["stale_critical"] == []
            and s["missing_critical"] == [],
            2.0, "fault with lidar unhealthy (not stale)", t0,
        )
        self.record("p_stale_stamp", latency_ms=lat * 1000, bound_ms=BOUND_S * 1000, detail=details[0],
                    treated_as="unhealthy_critical -> fault, MCU heartbeat withheld")
        self.assertLess(lat, BOUND_S)
        # Heartbeats kept flowing the whole time; the robot stays stopped while the stamps stay old.
        time.sleep(0.5)
        self.assertTrue(all(not nz for _, nz in self.cmds_since(t0 + lat)))
        # Fresh stamps again: stack recovers (re-arming) and motion resumes.
        t1 = time.monotonic()
        os.kill(self.pid("scan"), signal.SIGUSR1)
        self.wait_run(8.0)
        self.wait_stack(lambda s: s["state"] == "healthy", 2.0, "healthy again", t1)
