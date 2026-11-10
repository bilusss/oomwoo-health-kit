import signal
import time

import ohk_harness as h


def generate_test_description():
    return h.build_description(timer_beating=("critical_a",))


class TestStallTimerBeatUndetectable(h.OhkCase):
    def test_documents_undetectable_failure_not_desired_behaviour(self):
        """KNOWN GAP, not desired behaviour.

        critical_a beats from an independent timer (the anti-pattern the RFC warns about).
        Its work loop stalls, yet it keeps heartbeating, so the aggregator and stand-in
        cannot see the failure and the robot keeps moving.  This test pins that the
        failure is invisible; it passes because the monitor is blind, not because it is safe.
        """
        self.wait_run()
        t0 = time.monotonic()
        self.pid("critical_a")
        import os

        os.kill(self.pid("critical_a"), signal.SIGUSR1)
        # Prove the stall took effect: the timer beats now carry detail "stalled".
        def stalled_seen():
            with self.probe.lock:
                return any(c == "critical_a" and d == "stalled" and t >= t0 for t, c, d in self.probe.comp_details)

        self.wait_for(stalled_seen, 2.0, "stalled marker in critical_a heartbeats")
        t1 = time.monotonic()
        time.sleep(5.0)
        cmds = self.cmds_since(t1)
        self.assertGreater(len(cmds), 50)
        self.assertTrue(all(nz for _, nz in cmds), "motion stopped (monitor unexpectedly detected the stall)")
        self.assertEqual(self.states_since(t1), [])
        self.assertEqual(self.last_stack()["state"], "healthy")
        self.record("i_stall_timer_undetected", cmds=len(cmds))
