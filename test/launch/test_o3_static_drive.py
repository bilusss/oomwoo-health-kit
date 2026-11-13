"""Default design: static roster with drive supervision. Same staggered drive as scenarios o/o2 (commands 0.5 s after
the goal becomes active, silent 0.5 s before it ends); the roster does not follow the job, so nothing should change."""

import time

import ohk_nav_harness as n
from action_msgs.msg import GoalStatus


def generate_test_description():
    return n.build_nav_description(
        fakes=("scan", "odom", "tf", "polygon", "cmd"),
        # coasting after the last command (0.05 s) shorter than cmd_max_age (0.15 s) minus the command period (0.05 s)
        odom_extra=("--vx", "0.2", "--enable-topic", n.ENABLE_TOPIC, "--lag-s", "0.05"),
    )


class TestStaticDrive(n.NavCase):
    def test_goal_cycle_does_not_disturb_static_roster(self):
        self.wait_idle_run()
        t0 = self.set_goal(GoalStatus.STATUS_EXECUTING)
        time.sleep(0.5)
        self.set_enable(True)
        time.sleep(2.0)
        t_quiet = self.set_enable(False)
        time.sleep(0.5)
        self.set_goal(GoalStatus.STATUS_SUCCEEDED)
        time.sleep(1.5)
        t1 = time.monotonic()
        gaps = self.motion_gaps(t0, t1)
        stacks = {(s["state"], s["task_id"]) for _, s in self.stacks_since(t0)}
        self.record("o3_static_drive", zero_cmd_intervals_ms=[[round(a * 1000), round(b * 1000)] for a, b in gaps],
                    stack_states=sorted(stacks), mcu_transitions=len(self.states_since(t0)),
                    rosters_posted=len([1 for ts, _ in self.rosters if ts >= t0]), quiet_at_ms=round((t_quiet - t0) * 1000))
        self.assertEqual(stacks, {("healthy", "nav")})
        self.assertEqual(gaps, [])
        self.assertEqual(len(self.states_since(t0)), 0)
