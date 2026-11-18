"""ROS-free definitions for the M7 tasks of bench/fullstack: stack presets, scenario lists, simulator commands, log parsing.

Everything here is pure data or string building so that it can be unit-tested on the host (test/test_fullstack_tasks.py). The ROS
side (clients, subscriptions, subprocesses) is in jobs.py and run.py.

A TASK fixes the stack (localization, world, spawn pose, extra job nodes), the workload kind, the watcher and roster configs, and
which arming checks apply. The default task `nav_slam` is the M6 stack and workload, unchanged.
"""

import math
import os
import re

CONFIG = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "config")

LIVING_ROOM_SPAWN = (-2.0, -0.5, 0.0)
# kitchen_dining.world: vacuum_dock model at (-2.45, -0.5) facing +x; its mouth is at (-2.11, -0.5) and dock_sim.launch.py points the
# beacon along yaw 180 deg (into the bay). The robot reverses in, so it starts roughly facing the dock from the room side.
DOCK_SPAWN = (-1.5, -0.5, 3.14159)

TASKS = {
    "nav_slam": dict(localization="slam", world="living_room.world", spawn=LIVING_ROOM_SPAWN, job="none", workload="goals",
                     watch="nav_watch_sim.yaml", roster="nav_rosters.yaml", nav=True, planner=None),
    "nav_amcl": dict(localization="amcl", world="living_room.world", spawn=LIVING_ROOM_SPAWN, job="none", workload="goals",
                     watch="nav_watch_amcl.yaml", roster="nav_rosters_amcl.yaml", nav=True, planner=None),
    "coverage": dict(localization="amcl", world="living_room.world", spawn=LIVING_ROOM_SPAWN, job="coverage_support",
                     workload="coverage", watch="nav_watch_amcl.yaml", roster="nav_rosters_amcl.yaml", nav=True, planner="nav2"),
    "coverage_reactive": dict(localization="amcl", world="living_room.world", spawn=LIVING_ROOM_SPAWN, job="coverage_support",
                              workload="coverage", watch="nav_watch_amcl.yaml", roster="nav_rosters_amcl.yaml", nav=True,
                              planner="reactive"),
    # PROPOSAL variant (see config/nav_watch_amcl_reactive.yaml): the reactive executor commands at 5 Hz, slower than cmd_max_age 0.15 s
    "coverage_reactive_cmd30": dict(localization="amcl", world="living_room.world", spawn=LIVING_ROOM_SPAWN, job="coverage_support",
                                    workload="coverage", watch="nav_watch_amcl_reactive.yaml", roster="nav_rosters_amcl.yaml", nav=True,
                                    planner="reactive"),
    # M8: the same stacks and workloads with a watcher config that selects a new rule (opt-in; the M7 tasks above are unchanged)
    "nav_amcl_chain": dict(localization="amcl", world="living_room.world", spawn=LIVING_ROOM_SPAWN, job="none", workload="goals",
                           watch="nav_watch_amcl_chain.yaml", roster="nav_rosters_amcl_chain.yaml", nav=True, planner=None),
    "recovery_chain": dict(localization="amcl", world="living_room.world", spawn=LIVING_ROOM_SPAWN, job="none", workload="recovery",
                           watch="nav_watch_amcl_chain.yaml", roster="nav_rosters_amcl_chain.yaml", nav=True, planner=None),
    # M8 fault trials for behavior_server: long direct spin / backup actions after a goal (jobs.RecoveryProbeJob); `recovery_probe` is the
    # control with the M7 watch set, `recovery_probe_chain` selects rule A
    "recovery_probe": dict(localization="amcl", world="living_room.world", spawn=LIVING_ROOM_SPAWN, job="none", workload="recovery_probe",
                           watch="nav_watch_amcl.yaml", roster="nav_rosters_amcl.yaml", nav=True, planner=None),
    "recovery_probe_chain": dict(localization="amcl", world="living_room.world", spawn=LIVING_ROOM_SPAWN, job="none", workload="recovery_probe",
                                 watch="nav_watch_amcl_chain.yaml", roster="nav_rosters_amcl_chain.yaml", nav=True, planner=None),
    "dock_fix": dict(localization="none", world="kitchen_dining.world", spawn=DOCK_SPAWN, job="dock", workload="dock",
                     watch="nav_watch_dock_fix.yaml", roster="nav_rosters_dock_fix.yaml", nav=False, planner=None),
    "dock": dict(localization="none", world="kitchen_dining.world", spawn=DOCK_SPAWN, job="dock", workload="dock",
                 watch="nav_watch_dock.yaml", roster="nav_rosters_dock.yaml", nav=False, planner=None),
    # M9: in-node heartbeats (oomwoo_health_beat overlay). inode=<component ids the task's roster lists as critical>; needs --health-clock sim.
    # The `*_recon` tasks list the same components with a generous max_age_sec (5 s) so that a recon session measures the beat periods
    # without the loop reacting; the others carry the per-component max_age_sec derived from that recon (docs/M9_RECON.md).
    "dock_inode_recon": dict(localization="none", world="kitchen_dining.world", spawn=DOCK_SPAWN, job="dock", workload="dock",
                             watch="nav_watch_dock.yaml", roster="nav_rosters_dock_inode_recon.yaml", nav=False, planner=None,
                             inode=("dock_detector", "dock_drive")),
    "dock_inode": dict(localization="none", world="kitchen_dining.world", spawn=DOCK_SPAWN, job="dock", workload="dock",
                       watch="nav_watch_dock.yaml", roster="nav_rosters_dock_inode.yaml", nav=False, planner=None,
                       inode=("dock_detector", "dock_drive")),
    "coverage_inode_recon": dict(localization="amcl", world="living_room.world", spawn=LIVING_ROOM_SPAWN, job="coverage_support",
                                 workload="coverage", watch="nav_watch_amcl.yaml", roster="nav_rosters_amcl_inode_recon.yaml", nav=True,
                                 planner="nav2", inode=("coverage_planner",)),
    "coverage_inode": dict(localization="amcl", world="living_room.world", spawn=LIVING_ROOM_SPAWN, job="coverage_support",
                           workload="coverage", watch="nav_watch_amcl.yaml", roster="nav_rosters_amcl_inode.yaml", nav=True,
                           planner="nav2", inode=("coverage_planner",)),
    # reactive executor: the proposal watch config of M7 (cmd_max_age_s 0.30; the shipped 0.15 s gives ~490 stops per active hour here)
    "coverage_reactive_inode_recon": dict(localization="amcl", world="living_room.world", spawn=LIVING_ROOM_SPAWN, job="coverage_support",
                                          workload="coverage", watch="nav_watch_amcl_reactive.yaml",
                                          roster="nav_rosters_amcl_inode_recon.yaml", nav=True, planner="reactive",
                                          inode=("coverage_planner",)),
    "coverage_reactive_inode": dict(localization="amcl", world="living_room.world", spawn=LIVING_ROOM_SPAWN, job="coverage_support",
                                    workload="coverage", watch="nav_watch_amcl_reactive.yaml", roster="nav_rosters_amcl_inode.yaml",
                                    nav=True, planner="reactive", inode=("coverage_planner",)),
    "recovery": dict(localization="amcl", world="living_room.world", spawn=LIVING_ROOM_SPAWN, job="none", workload="recovery",
                     watch="nav_watch_amcl.yaml", roster="nav_rosters_amcl.yaml", nav=True, planner=None),
}


def launch_args(task, composition="true"):
    """`ros2 launch stack.launch.py` arguments (list of 'k:=v') that select the task's stack."""
    t = TASKS[task]
    x, y, yaw = t["spawn"]
    inode = "true" if t.get("inode") else "false"
    return [f"inode_health:={inode}", f"localization:={t['localization']}", f"world:={t['world']}", f"job:={t['job']}", f"x_pose:={x}", f"y_pose:={y}",
            f"yaw:={yaw}", f"composition:={composition}",
            f"watch_config:={os.path.join(CONFIG, t['watch'])}", f"roster_config:={os.path.join(CONFIG, t['roster'])}"]


# --------------------------------------------------------------------------- docking
# (name, x, y, yaw) in kitchen_dining world coordinates. The dock axis is y = -0.5; yaw pi = facing the dock (tail to the room
# side is wrong: the robot must turn and reverse in, which the dock_drive state machine does). "away" starts facing the room so the
# search spin runs first. Clearance: the dining table starts at y = -1.05, the robot radius is 0.1745.
DOCK_STARTS = [
    ("center", -1.50, -0.50, math.pi),
    ("left15", -1.50, -0.35, math.pi),
    ("right15", -1.50, -0.65, math.pi),
    ("yaw+17", -1.40, -0.50, math.pi - 0.3),
    ("yaw-17", -1.40, -0.50, -math.pi + 0.3),
    ("away", -1.50, -0.50, 0.0),
]
DOCK_ATTEMPT_TIMEOUT_SIM_S = 150.0
DOCK_TERMINAL = {"DONE": "succeeded", "IDLE": "gave_up"}


# --------------------------------------------------------------------------- obstacles for BT recoveries
WALL_SDF = ('<?xml version="1.0"?><sdf version="1.8"><model name="{name}"><static>true</static><link name="link">'
            '<collision name="c"><geometry><box><size>{size}</size></box></geometry></collision>'
            '<visual name="v"><geometry><box><size>{size}</size></box></geometry><material><ambient>0.85 0.1 0.1 1</ambient>'
            '<diffuse>0.85 0.1 0.1 1</diffuse></material></visual></link></model></sdf>')


def wall_sdf(name, length=2.4, thickness=0.05, height=1.0):
    """A static box the map does not know (same construction as oomwoo_sim_support spawn_obstacle.launch.py): the gpu_lidar sees it
    through its visual, the planner's costmaps mark it as an obstacle."""
    return WALL_SDF.format(name=name, size=f"{length:g} {thickness:g} {height:g}")


def wall_ahead(robot_xy, goal_xy, distance=0.7):
    """Wall pose across the straight line robot -> goal, `distance` m ahead of the robot: (x, y, yaw of the wall's long axis)."""
    ang = math.atan2(goal_xy[1] - robot_xy[1], goal_xy[0] - robot_xy[0])
    return (robot_xy[0] + distance * math.cos(ang), robot_xy[1] + distance * math.sin(ang), ang + math.pi / 2)


# (goal index into workload.POSES, wall: None | "persistent" | "transient"). persistent = the wall stays until the goal has ended
# (the BT runs out of retries or the goal is cancelled after RECOVERY_GOAL_MAX_SIM_S); transient = removed TRANSIENT_AFTER_SIM_S
# after it was placed, so the BT recoveries can end with a successful goal.
RECOVERY_CYCLE = [(1, "persistent"), (2, "transient"), (3, None), (4, "persistent"), (5, "transient"), (6, None), (7, "transient"),
                  (0, "persistent")]
WALL_DELAY_SIM_S = 4.0
TRANSIENT_AFTER_SIM_S = 8.0
RECOVERY_GOAL_MAX_SIM_S = 70.0
MIN_GOAL_DISTANCE_M = 1.8      # a wall placed 0.7 m ahead must still leave the goal beyond it

BT_RECOVERY_ACTIONS = ("spin", "backup", "wait", "drive_on_heading")
TERMINAL_STATUS = {4: "succeeded", 5: "canceled", 6: "aborted"}


class ActionTally:
    """Counts goals per action name from GoalStatusArray messages ([(goal_id, status)]); records (time, name, goal_id, first status).

    A goal is counted once as 'started' when first seen in a non-terminal status (accepted/executing/canceling) and once as its
    terminal status. A goal first seen already terminal (latched status after a late subscription) is not counted as started.
    """

    def __init__(self):
        self.seen = {}      # (name, goal_id) -> last status
        self.started = []   # (t, name, goal_id)
        self.counts = {}    # name -> {"started": n, "succeeded": n, "canceled": n, "aborted": n}

    def update(self, t, name, status_list):
        c = self.counts.setdefault(name, {"started": 0, "succeeded": 0, "canceled": 0, "aborted": 0})
        for gid, st in status_list:
            key = (name, gid)
            prev = self.seen.get(key)
            if prev == st:
                continue
            self.seen[key] = st
            if prev is None and st in (1, 2, 3):
                c["started"] += 1
                self.started.append((t, name, gid))
            if st in TERMINAL_STATUS and prev is not None and prev not in TERMINAL_STATUS:
                c[TERMINAL_STATUS[st]] += 1

    def started_between(self, t0, t1, names=BT_RECOVERY_ACTIONS):
        out = {}
        for t, name, _gid in self.started:
            if t0 <= t <= t1 and name in names:
                out[name] = out.get(name, 0) + 1
        return out


# --------------------------------------------------------------------------- coverage planner log
PLANNER_WP = re.compile(r"waypoint (\d+)/(\d+), coverage ([\d.]+)%")
PLANNER_PLAN = re.compile(r"coverage plan: (\d+) waypoints")
PLANNER_SKIP = re.compile(r"waypoint (\d+) (stuck \(no progress\); skipping|unreachable after \d+ tries; skipping)")


def parse_planner_log(lines):
    """Progress facts from the coverage_planner's own log: waypoints planned, last waypoint index, coverage %, stuck/unreachable
    waypoints, and whether it reports completion ('coverage complete')."""
    out = {"waypoints": None, "last_waypoint": None, "coverage_pct": None, "stuck": 0, "unreachable": 0, "complete": False}
    for ln in lines:
        m = PLANNER_PLAN.search(ln)
        if m:
            out["waypoints"] = int(m.group(1))
        m = PLANNER_WP.search(ln)
        if m:
            out["last_waypoint"], out["waypoints"] = int(m.group(1)), int(m.group(2))
            out["coverage_pct"] = float(m.group(3))
        m = PLANNER_SKIP.search(ln)
        if m:
            out["stuck" if m.group(2).startswith("stuck") else "unreachable"] += 1
        if "coverage complete" in ln.lower():
            out["complete"] = True
    return out


# --------------------------------------------------------------------------- fault targets (non-composed Nav2 + job nodes)
# fragment = substring of argv[0] (+ argv[1] for python scripts); resume = SIGSTOP faults are resumed after the hold time.
# `when` names the condition under which the trial signals (run.py when_ok): moving (>= 0.12 m/s) | fast (>= 0.30 m/s) | recovery | recovery_nominal | dock_back.
NOMINAL_YAW_RATE, NOMINAL_BACKUP_SPEED = 0.9, 0.10
WHEN_DWELL_S = {"recovery": 0.4, "recovery_nominal": 0.2}   # seconds the condition must hold before the signal (default 2.0); spin / backup last only a few seconds
M7_FAULTS = {
    "sigkill_amcl": ("/nav2_amcl/amcl", "SIGKILL", False, "moving"),
    "sigstop_amcl": ("/nav2_amcl/amcl", "SIGSTOP", True, "moving"),
    "sigkill_controller": ("/nav2_controller/controller_server", "SIGKILL", False, "moving"),
    "sigstop_controller": ("/nav2_controller/controller_server", "SIGSTOP", True, "moving"),
    "sigkill_coverage": ("/oomwoo_coverage/coverage_planner", "SIGKILL", False, "moving"),
    "sigstop_coverage": ("/oomwoo_coverage/coverage_planner", "SIGSTOP", True, "moving"),
    # reactive executor: signal only while the planner itself drives (v_cruise 0.35 m/s; Nav2 transits are capped at 0.2 m/s)
    "sigkill_coverage_fast": ("/oomwoo_coverage/coverage_planner", "SIGKILL", False, "fast"),
    "sigstop_coverage_fast": ("/oomwoo_coverage/coverage_planner", "SIGSTOP", True, "fast"),
    "sigkill_behavior": ("/nav2_behaviors/behavior_server", "SIGKILL", False, "recovery"),
    "sigstop_behavior": ("/nav2_behaviors/behavior_server", "SIGSTOP", True, "recovery"),
    # M8: the behavior_server fault triggered only at the nominal spin / backup rate (the exposure criterion of summarize_faults.py)
    "sigkill_behavior_nominal": ("/nav2_behaviors/behavior_server", "SIGKILL", False, "recovery_nominal"),
    "sigstop_behavior_nominal": ("/nav2_behaviors/behavior_server", "SIGSTOP", True, "recovery_nominal"),
    "sigkill_dock_drive": ("/oomwoo_dock/dock_drive", "SIGKILL", False, "dock_back"),
    "sigstop_dock_drive": ("/oomwoo_dock/dock_drive", "SIGSTOP", True, "dock_back"),
    "sigkill_dock_detector": ("/oomwoo_dock/dock_detector", "SIGKILL", False, "dock_back"),
    "sigstop_dock_detector": ("/oomwoo_dock/dock_detector", "SIGSTOP", True, "dock_back"),
}


def when_ok(when, speed, yaw_rate, recovery_active=False, dock_state=None):
    """Signal condition of a fault trial on the ground truth at one instant (speed m/s, |yaw rate| rad/s)."""
    if when == "moving":
        return speed >= 0.12
    if when == "fast":
        return speed >= 0.30
    if when == "recovery":   # a behavior_server action (spin / backup / wait) is active AND the robot is moving or turning
        return bool(recovery_active) and (speed >= 0.03 or yaw_rate >= 0.2)
    if when == "recovery_nominal":   # M8: an action is active AND it runs at its nominal rate: spin >= 0.9 rad/s or backup >= 0.10 m/s
        return bool(recovery_active) and (yaw_rate >= NOMINAL_YAW_RATE or speed >= NOMINAL_BACKUP_SPEED)
    if when == "dock_back":  # dock_drive is reversing into the bay
        return dock_state == "BACK" and speed >= 0.03
    return speed >= 0.12
