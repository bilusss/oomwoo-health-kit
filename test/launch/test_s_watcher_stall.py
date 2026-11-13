"""A stalled watcher must not turn queued (old) messages into fresh heartbeats."""

import os
import signal
import time

import ohk_nav_harness as n


def generate_test_description():
    return n.build_nav_description()


class TestWatcherStall(n.NavCase):
    def test_queued_messages_give_no_heartbeat(self):
        self.wait_idle_run()
        os.kill(self.pid("topic_heartbeat"), signal.SIGSTOP)
        time.sleep(0.3)  # the sources keep filling the (depth 10) subscription queues
        for k in ("scan", "odom", "tf", "polygon"):
            os.kill(self.pid(k), signal.SIGSTOP)
        time.sleep(1.0)
        t_cont = time.monotonic()
        os.kill(self.pid("topic_heartbeat"), signal.SIGCONT)
        time.sleep(1.0)
        with self.probe.lock:
            beats = [(round((ts - t_cont) * 1000), c) for ts, c, _ in self.probe.comp_details if ts >= t_cont]
        self.record("s_watcher_stall", stalled_s=1.0, heartbeats_after_cont=len(beats), first=beats[:5])
        self.assertEqual(beats, [], "queued messages were turned into heartbeats")
        self.assertTrue(all(not nz for _, nz in self.cmds_since(t_cont + 0.3)))
