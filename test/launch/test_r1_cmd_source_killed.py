"""Static roster: robot keeps moving (odom) while the command source dies -> stop; robot halts -> recovery."""

import os
import signal
import time

import ohk_harness as h
import ohk_nav_harness as n

CMD_MAX_AGE_S = 0.15
LIMITER_SLOT_S = 0.04  # drive_supervision emits at most 25 Hz
# worst case from the kill: command age reaches the limit, last heartbeat (one limiter slot later), aggregator
# max_age, aggregator tick, stand-in soft stop, stand-in tick, margin.
BOUND_S = CMD_MAX_AGE_S + LIMITER_SLOT_S + 0.15 + 0.05 + h.SOFT_STOP_S + h.TICK_S + h.MARGIN_S


def generate_test_description():
    return n.build_nav_description(fakes=("scan", "odom", "tf", "polygon", "cmd"), odom_extra=("--vx", "0.2"))


class TestCmdSourceKilled(n.NavCase):
    def test_moving_without_commands_stops(self):
        self.set_enable(True)
        self.wait_idle_run()  # moving, commands fresh
        t0, lat = self.signal_and_time_stop("cmd", signal.SIGKILL)
        self.record("r1_cmd_source_killed", latency_ms=lat * 1000, bound_ms=BOUND_S * 1000,
                    formula="cmd_max_age 0.15 + limiter 0.04 + max_age 0.15 + agg tick 0.05 + soft stop 0.15 + tick 0.01 + margin 0.1")
        self.assertLess(lat, BOUND_S)
        self.assert_blames("drive_supervision", t0)
        time.sleep(0.5)
        self.assertTrue(all(not nz for _, nz in self.cmds_since(t0 + lat)))
        t1 = time.monotonic()
        os.kill(self.pid("odom"), signal.SIGUSR2)  # the robot has stopped
        self.wait_run(8.0)
        self.wait_stack(lambda s: s["state"] == "healthy", 2.0, "healthy again", t1)
