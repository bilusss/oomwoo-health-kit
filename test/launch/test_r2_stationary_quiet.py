"""Static roster: stationary robot and no commander at all stays healthy (nothing to supervise). Then a commander holds a
non-zero setpoint and dies while odometry reads zero (stalled wheels): that must stop the robot, so the test fails if
drive_supervision always beat."""

import signal
import time

import ohk_harness as h
import ohk_nav_harness as n

CMD_MAX_AGE_S = 0.15
BOUND_S = CMD_MAX_AGE_S + 0.04 + 0.15 + 0.05 + h.SOFT_STOP_S + h.TICK_S + h.MARGIN_S


def generate_test_description():
    return n.build_nav_description(fakes=("scan", "odom", "tf", "polygon", "cmd"))  # odom reads zero all the time


class TestStationaryQuiet(n.NavCase):
    PROBE_FEED = False

    def test_idle_forever_then_held_setpoint_without_commander_stops(self):
        self.wait_quiet_run()
        t0 = time.monotonic()
        time.sleep(10.0)
        stacks = [s for _, s in self.stacks_since(t0)]
        with self.probe.lock:
            beats = sum(1 for ts, c, _ in self.probe.comp_details if c == "drive_supervision" and ts >= t0)
        self.record("r2_stationary_quiet", seconds=10, drive_supervision_beats=beats,
                    stack_states=sorted({s["state"] for s in stacks}))
        self.assertGreater(beats, 100, "drive_supervision must beat while idle (stationary, no setpoint)")
        self.assertTrue(all(s["state"].startswith("healthy") for s in stacks))
        self.assertEqual(self.states_since(t0), [])
        self.assertEqual(self.cmds_since(t0), [], "nothing feeds the stand-in, so nothing is gated")
        # phase 2: held non-zero setpoint, commander killed, odometry still reads zero
        self.set_enable(True)
        self.wait_run(8.0)
        t1, lat = self.signal_and_time_stop("cmd", signal.SIGKILL)
        self.record("r2_held_setpoint_dead_commander", latency_ms=lat * 1000, bound_ms=BOUND_S * 1000)
        self.assertLess(lat, BOUND_S)
        self.assert_blames("drive_supervision", t1)
        time.sleep(2.0)
        self.assertNotEqual(self.current_state(), "run")
