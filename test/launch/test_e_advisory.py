import signal
import time

import ohk_harness as h


def generate_test_description():
    return h.build_description()


class TestAdvisory(h.OhkCase):
    def test_advisory_killed_motion_continues(self):
        self.wait_run()
        import os

        t0 = time.monotonic()
        os.kill(self.pid("advisory_a"), signal.SIGKILL)
        time.sleep(5.0)
        cmds = self.cmds_since(t0)
        self.assertGreater(len(cmds), 50)
        self.assertTrue(all(nz for _, nz in cmds), "zero cmd_vel after advisory loss")
        self.assertEqual(len(self.states_since(t0)), 0)
        self.wait_stack(
            lambda s: s["advisory_faults"] == ["advisory_a"] and s["state"] == "healthy_with_advisory_faults"
            and s["emit_mcu_heartbeat"],
            1.0,
            "advisory fault in stack",
            t0,
        )
        self.record("e_advisory", cmds=len(cmds))
