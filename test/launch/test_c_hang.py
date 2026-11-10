"""Scenario c: process frozen (SIGSTOP)."""

import os
import signal
import time

import ohk_harness as h


def generate_test_description():
    return h.build_description()


class TestProcessFrozen(h.OhkCase):
    def test_sigstop_then_resume(self):
        self.wait_run()
        t0, lat = self.signal_and_time_stop("critical_a", signal.SIGSTOP)
        self.record("c_hang", latency_ms=lat * 1000)
        self.assertLess(lat, h.FAULT_TO_STOP_MAX_S)
        self.assert_blames("critical_a", t0)
        # stay frozen well short of CPU_RESET; motion must stay zero
        time.sleep(0.8)
        self.assertTrue(all(not nz for _, nz in self.cmds_since(t0 + lat)))
        t_cont = time.monotonic()
        os.kill(self.pid("critical_a"), signal.SIGCONT)
        self.wait_for(lambda: any(nz for _, nz in self.cmds_since(t_cont)), 10, "motion after SIGCONT")
        first_move = next(t for t, nz in self.cmds_since(t_cont) if nz)
        run_states = [(t, s) for t, s in self.states_since(t_cont) if s["state"] == "run"]
        self.assertTrue(run_states, "motion without a RUN state report")
        self.assertLessEqual(run_states[0][0], first_move)
        self.assertIn("consecutive heartbeats", run_states[0][1]["reason"])
        # Observed through the 20 Hz probe, so quantised to ~50 ms.
        self.record(
            "c_hang_rearm", rearm_ms=(first_move - t_cont) * 1000, note="quantised to the 20 Hz probe rate"
        )
