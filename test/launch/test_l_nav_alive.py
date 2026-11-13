import time

import ohk_nav_harness as n


def generate_test_description():
    return n.build_nav_description()


class TestNavAlive(n.NavCase):
    def test_all_topics_alive(self):
        self.wait_idle_run()
        t0 = time.monotonic()
        time.sleep(3.0)
        cmds = self.cmds_since(t0)
        self.assertGreater(len(cmds), 40)
        self.assertTrue(all(nz for _, nz in cmds), "cmd_vel blocked while everything is alive")
        self.assertEqual(self.states_since(t0), [], "stand-in left RUN")
        stacks = [s for _, s in self.stacks_since(t0)]
        self.assertTrue(all(s["state"] == "healthy" and s["task_id"] == "nav" for s in stacks))
        with self.probe.lock:
            ids = [c for ts, c, _ in self.probe.comp_details if ts >= t0]
        rates = {c: ids.count(c) / 3.0 for c in set(ids)}
        self.record("l_nav_alive", cmds=len(cmds), heartbeat_hz={k: round(v, 1) for k, v in sorted(rates.items())})
        # Fake rates: scan 5, odom 50 (unlimited), tf 50 (limited to 10), costmap 5; stationary robot, no commands:
        # drive_supervision beats from the odom callbacks, limited to 25 Hz; drive_path is not in the static roster.
        self.assertTrue(4.0 <= rates["lidar"] <= 5.6, rates)
        self.assertTrue(40.0 <= rates["odom"] <= 52.0, rates)
        self.assertTrue(20.0 <= rates["drive_supervision"] <= 27.0, rates)
        self.assertTrue(8.0 <= rates["localization"] <= 11.0, rates)
        self.assertTrue(4.0 <= rates["local_costmap"] <= 5.6, rates)
        self.assertNotIn("drive_path", rates)
