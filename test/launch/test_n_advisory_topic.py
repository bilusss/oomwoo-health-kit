import signal
import time

import ohk_nav_harness as n


def generate_test_description():
    return n.build_nav_description()


class TestAdvisoryTopic(n.NavCase):
    def test_costmap_topic_stops_motion_continues(self):
        self.wait_idle_run()
        t0 = time.monotonic()
        import os

        os.kill(self.pid("polygon"), signal.SIGKILL)
        time.sleep(3.0)
        cmds = self.cmds_since(t0)
        self.assertGreater(len(cmds), 40)
        self.assertTrue(all(nz for _, nz in cmds), "zero cmd_vel after advisory topic loss")
        self.assertEqual(len(self.states_since(t0)), 0)
        self.wait_stack(
            lambda s: s["advisory_faults"] == ["local_costmap"] and s["state"] == "healthy_with_advisory_faults"
            and s["emit_mcu_heartbeat"],
            1.0, "advisory fault in stack", t0,
        )
        self.record("n_advisory_topic", cmds=len(cmds))
