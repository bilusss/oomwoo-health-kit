"""A command that waited in the watcher's queue is not fresh, but it is still the setpoint the stand-in forwarded: the watcher
must not forget it. Commander holds a non-zero setpoint and dies while the watcher is stalled; odometry reads zero. After the
watcher resumes (queued commands are stale) the stack must stay stopped, not start beating on 'odometry is quiet'."""

import os
import signal
import time

import ohk_nav_harness as n


def generate_test_description():
    return n.build_nav_description(fakes=("scan", "odom", "tf", "polygon", "cmd"))


class TestStaleCommandSetpoint(n.NavCase):
    PROBE_FEED = False

    def test_stale_queued_command_keeps_the_setpoint(self):
        self.wait_quiet_run()  # no command ever seen, odometry quiet: healthy
        os.kill(self.pid("topic_heartbeat"), signal.SIGSTOP)
        time.sleep(0.1)
        self.set_enable(True)  # the commander starts and dies while the watcher is stalled: its commands only sit in the queue
        time.sleep(0.4)
        os.kill(self.pid("cmd"), signal.SIGKILL)
        time.sleep(0.6)  # queued commands are older than 1/25 s + slack
        t_cont = time.monotonic()
        os.kill(self.pid("topic_heartbeat"), signal.SIGCONT)
        time.sleep(3.0)
        with self.probe.lock:
            ds = [ts for ts, c, _ in self.probe.comp_details if c == "drive_supervision" and ts >= t_cont]
            lidar = [ts for ts, c, _ in self.probe.comp_details if c == "lidar" and ts >= t_cont]
        self.record("s2_stale_command_setpoint", drive_supervision_beats_after_cont=len(ds), lidar_beats_after_cont=len(lidar))
        self.assertGreater(len(lidar), 5, "the watcher must be running again")
        self.assertEqual(ds, [], "drive_supervision forgot the non-zero setpoint that sat in its queue")
        self.assertNotEqual(self.current_state(), "run")
