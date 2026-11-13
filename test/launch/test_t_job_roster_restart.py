"""Optional job roster: a dead job_roster stops the robot (plumbing heartbeat); a restart in the middle of a goal
posts the strictest roster first and never a relaxed one while the goal is active."""

import os
import signal
import subprocess
import time

import ohk_harness as h
import ohk_nav_harness as n
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_prefix

# job_roster max_age 0.5 -> aggregator tick -> soft stop -> margin
BOUND_S = 0.5 + 0.05 + h.SOFT_STOP_S + h.TICK_S + h.MARGIN_S


def generate_test_description():
    return n.build_nav_description(
        n.job_rosters(constant_ids=True), fakes=("scan", "odom", "tf", "polygon", "drive"),
        odom_extra=("--vx", "0.2", "--enable-topic", n.ENABLE_TOPIC),
    )


class TestJobRosterRestart(n.NavCase):
    def test_dead_job_roster_stops_and_restart_is_strict(self):
        self.wait_idle_run()
        self.set_goal(GoalStatus.STATUS_EXECUTING)
        time.sleep(0.5)
        self.set_enable(True)
        self.wait_run(8.0)
        self.wait_for(lambda: self.crit(self.last_roster(), "drive_path") is True, 2.0, "strict roster while driving")
        t0, lat = self.signal_and_time_stop("job_roster", signal.SIGKILL)
        self.record("t_job_roster_dead", latency_ms=lat * 1000, bound_ms=BOUND_S * 1000)
        self.assertLess(lat, BOUND_S)
        self.assert_blames("job_roster", t0)
        exe = os.path.join(get_package_prefix("oomwoo_health_kit"), "lib", "oomwoo_health_kit", "job_roster")
        t1 = time.monotonic()
        self.spawned.append(subprocess.Popen([exe, "--ros-args", "-p", f"config_file:={n.CONFIG_PATHS['rosters']}"]))
        self.wait_for(lambda: any(ts >= t1 for ts, _ in self.rosters), 5.0, "roster after restart")
        self.wait_run(10.0)  # goal still active, drive alive: strict roster, healthy
        after = [r for ts, r in self.rosters if ts >= t1]
        self.record("t_job_roster_restart", rosters_after_restart=len(after),
                    drive_path_critical=[self.crit(r, "drive_path") for r in after], recovered_s=time.monotonic() - t1)
        self.assertTrue(all(self.crit(r, "drive_path") for r in after), f"relaxed roster posted during an active goal: {[(round((ts - t1) * 1000), r) for ts, r in self.rosters if ts >= t1]}")
