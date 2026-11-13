"""Default design: static roster with drive supervision. Same staggered drive as scenarios o/o2 (commands 0.5 s after
the goal becomes active, a final zero command 0.5 s before it ends, then silence); the roster does not follow the job, so
nothing should change. Then the commander dies mid-drive (non-zero setpoint, robot still moving): the stop must follow,
so the test fails if drive_supervision always beat."""

import signal
import time

import ohk_harness as h

import ohk_nav_harness as n
from action_msgs.msg import GoalStatus


def generate_test_description():
    return n.build_nav_description(
        fakes=("scan", "odom", "tf", "polygon", "cmd"),
        # coasting after the last command (0.05 s) shorter than cmd_max_age (0.15 s) minus the command period (0.05 s)
        odom_extra=("--vx", "0.2", "--enable-topic", n.ENABLE_TOPIC, "--lag-s", "0.05"),
    )


class TestStaticDrive(n.NavCase):
    PROBE_FEED = False

    def test_goal_cycle_does_not_disturb_static_roster(self):
        self.wait_quiet_run()
        t0 = self.set_goal(GoalStatus.STATUS_EXECUTING)
        time.sleep(0.5)
        self.set_enable(True)
        time.sleep(2.0)
        t_quiet = self.set_enable(False)
        time.sleep(0.5)
        self.set_goal(GoalStatus.STATUS_SUCCEEDED)
        time.sleep(1.5)
        t1 = time.monotonic()
        stacks = {(s["state"][:7], s["task_id"]) for _, s in self.stacks_since(t0)}
        soft = [e for _, e in self.states_since(t0)]
        self.record("o3_static_drive", stack_states=sorted(stacks), mcu_transitions=len(soft),
                    rosters_posted=len([1 for ts, _ in self.rosters if ts >= t0]), quiet_at_ms=round((t_quiet - t0) * 1000))
        self.assertEqual(stacks, {("healthy", "nav")})
        self.assertEqual(soft, [])
        # commander dies while driving (odom keeps reporting motion, a non-zero setpoint is held): must stop
        self.set_enable(True)
        self.wait_run(8.0)
        t2, lat = self.signal_and_time_stop("cmd", signal.SIGKILL)
        self.record("o3_commander_dies_while_driving", latency_ms=lat * 1000)
        self.assertLess(lat, 0.15 + 0.04 + 0.15 + 0.05 + h.SOFT_STOP_S + h.TICK_S + h.MARGIN_S)
        self.assert_blames("drive_supervision", t2)
