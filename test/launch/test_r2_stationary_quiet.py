"""Static roster: stationary robot and no commands at all stays healthy (no job, nothing to supervise)."""

import time

import ohk_nav_harness as n


def generate_test_description():
    return n.build_nav_description()


class TestStationaryQuiet(n.NavCase):
    def test_idle_forever(self):
        self.wait_idle_run()
        t0 = time.monotonic()
        time.sleep(10.0)
        cmds = self.cmds_since(t0)
        stacks = [s for _, s in self.stacks_since(t0)]
        self.record("r2_stationary_quiet", seconds=10, cmds=len(cmds), stack_states=sorted({s["state"] for s in stacks}))
        self.assertGreater(len(cmds), 150)
        self.assertTrue(all(nz for _, nz in cmds))
        self.assertTrue(all(s["state"] == "healthy" for s in stacks))
        self.assertEqual(self.states_since(t0), [])
