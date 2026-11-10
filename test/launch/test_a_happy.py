import time
import unittest

import launch_testing
import ohk_harness as h


def generate_test_description():
    return h.build_description()


class TestHappy(h.OhkCase):
    def test_run_and_stay(self):
        self.wait_run()
        t0 = time.monotonic()
        n_before = len(self.probe.states)
        time.sleep(5.0)
        cmds = self.cmds_since(t0)
        self.assertGreater(len(cmds), 50)
        self.assertTrue(all(nz for _, nz in cmds), "zero cmd_vel while healthy")
        self.assertEqual(len(self.states_since(t0)), 0, "transition during steady RUN")
        self.assertEqual(self.current_state(), "run")
        self.record("a_happy", transitions_in_5s=len(self.states_since(t0)), n_before=n_before)
