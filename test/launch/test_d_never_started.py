import time

import ohk_harness as h


def generate_test_description():
    return h.build_description(
        critical=("critical_a", "critical_ghost"),
        started=["critical_a", "advisory_a"],
    )


class TestNeverStarted(h.OhkCase):
    def test_never_run(self):
        t0 = time.monotonic()
        time.sleep(5.0)
        cmds = self.cmds_since(t0)
        self.assertGreater(len(cmds), 50)
        self.assertTrue(all(not nz for _, nz in cmds))
        self.assertNotIn("run", [s["state"] for _, s in self.states_since(0)])
        stack = self.last_stack()
        self.assertEqual(stack["state"], "fault")
        self.assertEqual(stack["missing_critical"], ["critical_ghost"])
        self.assertEqual(self.faulted_critical(stack), [])
        self.record("d_never_started", cmds=len(cmds))
