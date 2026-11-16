"""M8 review fix: odometry shows motion, a fresh command keeps drive_supervision healthy, but the stage output topic never carries a message
(a remapped or silent smoother, or a writer that bypasses the stage). The rule must go not-ok after the motion grace instead of staying
healthy because 'nothing was ever seen'."""

import time

import ohk_nav_harness as n


def generate_test_description():
    return n.m8_description("chain", fakes=("scan", "odom", "tf", "polygon", "smoother", "commander"), odom_extra=("--vx", "0.2"))


class TestChainMotionWithoutOutput(n.NavCase):
    PROBE_FEED = True      # the probe writes a constant non-zero command on the stand-in input: drive_supervision stays healthy while odometry moves

    def test_motion_with_a_silent_stage_is_not_healthy(self):
        # the 'commander' and 'smoother' fakes are running (topics have publishers of the right type) but never enabled: no stage output at all
        t0 = time.monotonic()
        self.wait_for(lambda: any(s["state"] == "fault" and "command_chain" in self.faulted_critical(s) for _, s in self.stacks_since(t0)), 8.0,
                      "command_chain blamed while odometry moves and the stage is silent")
        self.assertFalse(any(s["state"].startswith("healthy") and s["emit_mcu_heartbeat"] for _, s in self.stacks_since(t0 + 3.0)), "never healthy")
        self.record("u8_chain_motion_without_output")
