"""A small held command (0.015 m/s, below the 0.02 m/s odometry threshold) with odometry reading zero: the commander dies,
the MCU keeps the setpoint. Old logic ("stationary = odom under threshold") kept the heartbeat alive forever; the setpoint
rule must stop the robot within the same bound as a fast command."""

import signal
import time

import ohk_harness as h
import ohk_nav_harness as n

BOUND_S = 0.15 + 0.04 + 0.15 + 0.05 + h.SOFT_STOP_S + h.TICK_S + h.MARGIN_S


def generate_test_description():
    return n.build_nav_description(fakes=("scan", "odom", "tf", "polygon"), odom_extra=("--vx", "0"))


class TestCreepSetpoint(n.NavCase):
    PROBE_FEED = False

    def test_creep_setpoint_dead_commander(self):
        self.spawn_fake("cmd", ("--value", "0.015"))
        self.set_enable(True)
        self.wait_run(10.0)
        self.wait_stack(lambda s: s["state"].startswith("healthy"), 3.0, "healthy while the commander is alive")
        t0 = time.monotonic()
        self.spawned[-1].send_signal(signal.SIGKILL)
        self.wait_for(lambda: self.first_zero_since(t0) is not None, BOUND_S + 1.0, "stop after the creep commander died")
        lat = self.first_zero_since(t0)
        self.record("r4_creep_setpoint", latency_ms=lat * 1000, bound_ms=BOUND_S * 1000)
        self.assertLess(lat, BOUND_S)
        self.assert_blames("drive_supervision", t0)
        time.sleep(3.0)
        self.assertNotEqual(self.current_state(), "run")
