import time

import ohk_harness as h


def generate_test_description():
    return h.build_description(roster=False)


class TestNoRoster(h.OhkCase):
    def test_never_run(self):
        t0 = time.monotonic()
        time.sleep(5.0)
        cmds = self.cmds_since(t0)
        self.assertGreater(len(cmds), 50)
        self.assertTrue(all(not nz for _, nz in cmds))
        self.assertNotIn("run", [s["state"] for _, s in self.states_since(0)])
        self.assertEqual(self.last_stack()["state"], "no_roster")
        self.record("g_no_roster", cmds=len(cmds))
