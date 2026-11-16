"""M8 rule B, fault case: the dock detector dies while the robot reverses on its odometry carry (dock_drive keeps commanding -0.06 m/s
for 3 s). Command stream stays fresh, so only the dock_fix component can stop the robot, within fix_max_age plus the usual latency."""

import signal
import time

import ohk_harness as h
import ohk_nav_harness as n

FIX_MAX_AGE_S = n.REVFIX_MAX_AGE_S      # from config/nav_watch_dock_fix.yaml
BOUND_S = FIX_MAX_AGE_S + 0.15 + 0.04 + h.SOFT_STOP_S + h.TICK_S + h.MARGIN_S


def generate_test_description():
    return n.m8_description("revfix", fakes=("scan", "odom", "tf", "polygon", "fix"))


class TestRevfixDeadDetector(n.NavCase):
    PROBE_FEED = False

    def test_dead_detector_while_reversing_is_detected(self):
        self.wait_quiet_run()
        self.spawn_fake("cmd", ("--value", "-0.06"))
        self.set_enable(True)
        self.wait_for(lambda: self.current_state() == "run" and any(nz for _, nz in self.cmds_since(time.monotonic() - 0.3)), 10.0,
                      "RUN with a reversing command")
        time.sleep(2.0)
        self.assertEqual(self.current_state(), "run", "no soft stop while the fixes arrive every 0.6 s")
        t0, lat = self.signal_and_time_stop("fix", signal.SIGKILL, timeout=BOUND_S + 1.0)
        self.record("u5_revfix_dead_detector", latency_ms=lat * 1000, bound_ms=BOUND_S * 1000)
        self.assertLess(lat, BOUND_S)
        self.assertGreater(lat, FIX_MAX_AGE_S - 0.6 - 0.05, "not before the bound could have expired")
        self.assert_blames("dock_fix", t0)
