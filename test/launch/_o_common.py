"""Shared body of the job-roster transition scenarios (o, o2): real Nav2 is silent while it plans, so the drive
commands start 0.5 s after the goal becomes active and stop 0.5 s before it ends."""

import time

import ohk_harness as h
import ohk_nav_harness as n
from action_msgs.msg import GoalStatus

POLL_S = 0.02
AGG_TICK_S = 0.05
# Stop expected at goal start: strict roster posted within one poll, aggregator tick, then stand-in soft stop.
START_STOP_MAX_S = POLL_S + AGG_TICK_S + h.SOFT_STOP_S + h.TICK_S + h.MARGIN_S
# Stop expected when the drive commands cease: drive_path max_age, aggregator tick, soft stop.
SILENCE_STOP_MAX_S = 0.15 + AGG_TICK_S + h.SOFT_STOP_S + h.TICK_S + h.MARGIN_S
HEALTHY_MAX_S = 4.0


def healthy(s):
    return s["state"].startswith("healthy")


class JobTransitions(n.NavCase):
    NAME = "o_job_transition"

    def measure(self, t0, label):
        t1 = time.monotonic()
        gaps = self.motion_gaps(t0, t1)
        first = lambda pred: next((round((ts - t0) * 1000) for ts, s in self.stacks_since(t0) if pred(s)), None)
        return {
            "first_fault_ms": first(lambda s: s["state"] == "fault"),
            "first_arming_ms": first(lambda s: s["state"] == "arming"),
            "fault_blamed": sorted({x for _, s in self.stacks_since(t0) for x in self.faulted_critical(s)
                                    + s["missing_critical"]}),
            "zero_cmd_intervals_ms": [[round(a * 1000), round(b * 1000)] for a, b in gaps],
            "motion_inhibited_ms": round(sum(b - a for a, b in gaps) * 1000),
            "mcu_transitions": [[e[0], e[2]] for e in self.timeline(t0) if e[1] == "mcu"],
            "stack_timeline": [list(e) for e in self.timeline(t0) if e[1] == "stack"][:14],
        }

    def run_until_settled(self, t0, label):
        self.wait_for(lambda: healthy(self.probe.stacks[-1][1]) and self.current_state() == "run"
                      and (lambda c: bool(c) and c[-1][1])(self.cmds_since(time.monotonic() - 0.3)),
                      HEALTHY_MAX_S, f"{label}: healthy, RUN, non-zero cmd_vel")

    def test_job_transitions(self):
        self.wait_idle_run()
        t_goal = self.set_goal(GoalStatus.STATUS_EXECUTING)  # Nav2 is planning: no commands yet
        time.sleep(0.5)
        t_drive = self.set_enable(True)
        self.run_until_settled(t_goal, "goal start")
        t_settled = time.monotonic()
        start = self.measure(t_goal, "goal start")
        start["settled_after_goal_ms"] = round((t_settled - t_goal) * 1000)
        start["first_zero_cmd_after_goal_ms"] = None if (z := self.first_zero_since(t_goal)) is None else round(z * 1000)
        start["drive_started_ms"] = round((t_drive - t_goal) * 1000)
        self.assertIsNotNone(z, "strict roster with a silent drive path must stop the robot (contract)")
        self.assertLess(z, START_STOP_MAX_S)
        time.sleep(1.0)
        t_quiet = self.set_enable(False)  # commands stop 0.5 s before the goal ends
        # once settled, motion is not interrupted while driving (the start-of-goal stop above is the documented cost of this
        # optional roster; restart of arming at the transition is allowed only before t_settled)
        self.assertEqual(self.motion_gaps(t_settled, t_quiet), [], "motion interrupted while driving")
        silence_stop = None
        self.wait_for(lambda: self.first_zero_since(t_quiet) is not None, 2.0, "stop after the drive path went silent")
        silence_stop = self.first_zero_since(t_quiet)
        self.assertLess(silence_stop, SILENCE_STOP_MAX_S)
        time.sleep(max(0.0, 0.5 - (time.monotonic() - t_quiet)))
        t_end = self.set_goal(GoalStatus.STATUS_SUCCEEDED)
        self.run_until_settled(t_end, "goal end")
        end = self.measure(t_quiet, "goal end")
        end["silence_to_stop_ms"] = round(silence_stop * 1000)
        end["goal_end_to_settled_ms"] = round((time.monotonic() - t_end) * 1000)
        end["relaxed_after_goal_end_ms"] = next(
            (round((ts - t_end) * 1000) for ts, r in list(self.rosters) if ts >= t_end), None)
        self.record(self.NAME, goal_start=start, goal_end_window=end,
                    bounds_ms={"start_stop": START_STOP_MAX_S * 1000, "silence_stop": SILENCE_STOP_MAX_S * 1000})
        self.assertNotIn("cpu_reset", [s["state"] for _, s in self.probe.states])
