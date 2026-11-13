"""Launch description + helpers for the topic-watcher scenarios (l-p): fake real-node publishers, no Gazebo."""

import json
import os
import subprocess
import sys
import tempfile
import time

import launch
import launch_testing.actions
import ohk_harness as h
import rclpy
import yaml
from action_msgs.msg import GoalStatus, GoalStatusArray
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from ohk_harness import latched_state_qos
from std_msgs.msg import String

STATUS_TOPIC = "/navigate_to_pose/_action/status"
FAKES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ohk_fakes.py")

ENABLE_TOPIC = "/fake/enable"
# Test copy of config/nav_*.yaml: same components and max_age values, test topic names. /nav_cmd stands for the stand-in's
# command input: the commander (fake "cmd", or the Probe's constant feed) writes cmd_vel_in, the topic the stand-in receives, and
# drive_supervision watches exactly that topic. /drive_cmd stands for the single-writer drive-chain output (drive_path).
SCAN_MAX_AGE = 0.6  # 3 x 0.2 s (5 Hz)
WATCHES = {
    "watches": [
        {"component_id": "lidar", "topic": "/scan", "type": "sensor_msgs/msg/LaserScan", "stamp_max_age_s": 0.5},
        {"component_id": "odom", "topic": "/odom", "type": "nav_msgs/msg/Odometry", "max_emit_hz": 50},
        {"component_id": "localization", "tf": {"parent": "map", "child": "odom"}, "max_emit_hz": 10},
        {"component_id": "drive_supervision", "max_emit_hz": 25,
         "motion_consistency": {"odom_topic": "/odom", "cmd_topic": "/cmd_vel_in", "cmd_max_age_s": 0.15}},
        {"component_id": "drive_path", "topic": "/drive_cmd", "type": "geometry_msgs/msg/Twist"},
        {"component_id": "local_costmap", "topic": "/local_fp", "type": "geometry_msgs/msg/PolygonStamped"},
    ]
}
BASE = [
    {"component_id": "lidar", "critical": True, "max_age_sec": SCAN_MAX_AGE},
    {"component_id": "odom", "critical": True, "max_age_sec": 0.15},
    {"component_id": "localization", "critical": True, "max_age_sec": 1.0},
    {"component_id": "local_costmap", "critical": False, "max_age_sec": 0.6},
]
SUPERVISION = {"component_id": "drive_supervision", "critical": True, "max_age_sec": 0.15}
DRIVE = {"component_id": "drive_path", "critical": True, "max_age_sec": 0.15}


def static_roster():
    """The default design: one static roster, drive supervised by motion consistency."""
    return {"state_source": {"type": "static"},
            "rosters": {"nav": {"task_id": "nav", "components": [*BASE, SUPERVISION]}}}


def job_rosters(constant_ids=True, source="nav2_actions"):
    """Optional job roster. `constant_ids`: one task_id and the same ids (drive_path advisory while idle);
    otherwise idle lacks drive_path, task_ids differ and allow_arming_restart is set (measures the restart cost)."""
    if constant_ids:
        idle = {"task_id": "nav", "components": [*BASE, {**DRIVE, "critical": False}]}
        drv = {"task_id": "nav", "components": [*BASE, DRIVE]}
    else:
        idle = {"task_id": "nav_idle", "components": BASE}
        drv = {"task_id": "nav_driving", "components": [*BASE, DRIVE]}
    src = {"type": "nav2_actions", "status_topics": [STATUS_TOPIC], "settle_s": 3.0} if source == "nav2_actions" \
        else {"type": "topic"}
    return {"debounce_s": 0.3, "state_source": src, "allow_arming_restart": not constant_ids,
            "rosters": {"idle": idle, "driving": drv}}


def write_yaml(doc, name):
    path = os.path.join(tempfile.mkdtemp(prefix="ohk_cfg_"), name)
    with open(path, "w") as f:
        yaml.safe_dump(doc, f)
    return path


CONFIG_PATHS = {}


def fake_cmd(kind, name):
    return {
        "scan": ["--kind", "scan", "--topic", "/scan", "--hz", "5"],
        "odom": ["--kind", "odom", "--topic", "/odom", "--hz", "50"],
        "tf": ["--kind", "tf", "--topic", "/tf", "--hz", "50"],
        "polygon": ["--kind", "polygon", "--topic", "/local_fp", "--hz", "5"],
        # silent while "off" on ENABLE_TOPIC; the staggered drive of scenario o/o3 is driven by the test
        "drive": ["--kind", "twist", "--topic", "/drive_cmd", "--hz", "20", "--enable-topic", ENABLE_TOPIC],
        "cmd": ["--kind", "twist", "--topic", "/cmd_vel_in", "--hz", "20", "--enable-topic", ENABLE_TOPIC],
    }[kind]


def fake_process(kind, extra=()):
    return ExecuteProcess(cmd=[sys.executable, FAKES, *fake_cmd(kind, kind), *extra], name=kind, output="screen")


def build_nav_description(roster_doc=None, fakes=("scan", "odom", "tf", "polygon"), odom_extra=("--vx", "0")):
    """Default: static roster; stationary robot (odom vx 0), no command source."""
    h.ACTIONS.clear()
    ld = launch.LaunchDescription()

    def add(name, action):
        h.ACTIONS[name] = action
        ld.add_action(action)

    CONFIG_PATHS["rosters"] = write_yaml(roster_doc or static_roster(), "rosters.yaml")
    CONFIG_PATHS["watch"] = write_yaml(WATCHES, "watch.yaml")
    add("health_monitor", Node(name="health_monitor", package="oomwoo_health_monitor", executable="health_monitor_node"))
    add("mcu_standin", Node(name="mcu_standin", package="oomwoo_health_kit", executable="mcu_standin_node"))
    add("job_roster", Node(
        name="job_roster", package="oomwoo_health_kit", executable="job_roster",
        parameters=[{"config_file": CONFIG_PATHS["rosters"]}],
    ))
    add("topic_heartbeat", Node(
        name="topic_heartbeat", package="oomwoo_health_kit", executable="topic_heartbeat",
        parameters=[{"config_file": CONFIG_PATHS["watch"]}],
    ))
    for k in fakes:
        add(k, fake_process(k, odom_extra if k == "odom" else ()))
    ld.add_action(launch_testing.actions.ReadyToTest())
    return ld


class NavCase(h.OhkCase):
    """OhkCase plus a goal-status publisher and a job-state publisher living in the test process."""

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cls.probe.feed = getattr(cls, "PROBE_FEED", True)  # False: only the "cmd" fake (or a test) writes cmd_vel_in
        cls.rosters = []  # (monotonic_t, dict) every roster the probe sees
        cls.probe.create_subscription(
            String, "oomwoo/health/roster",
            lambda m: cls.rosters.append((time.monotonic(), json.loads(m.data))), latched_state_qos(),
        )
        cls.status_pub = cls.probe.create_publisher(GoalStatusArray, STATUS_TOPIC, qos)
        cls.job_pub = cls.probe.create_publisher(String, "oomwoo/health/job_state", qos)
        cls.enable_pub = cls.probe.create_publisher(String, ENABLE_TOPIC, qos)
        cls.spawned = []  # extra subprocesses started by a test; killed in tearDownClass

    @classmethod
    def tearDownClass(cls):
        for p in cls.spawned:
            try:
                p.kill()
            except Exception:
                pass
        super().tearDownClass()

    def spawn_fake(self, kind, extra=()):
        p = subprocess.Popen([sys.executable, FAKES, *fake_cmd(kind, kind), *extra])
        self.spawned.append(p)
        return p

    def set_enable(self, on):
        self.enable_pub.publish(String(data="on" if on else "off"))
        return time.monotonic()

    def last_roster(self):
        return self.rosters[-1][1] if self.rosters else None

    @staticmethod
    def crit(roster, cid):
        return next((c["critical"] for c in roster["components"] if c["component_id"] == cid), None)

    def first_zero_since(self, t):
        return next((ts - t for ts, nz in self.cmds_since(t) if not nz), None)

    def set_goal(self, status):
        """status: GoalStatus.STATUS_* of one goal, or None for an empty array."""
        m = GoalStatusArray()
        if status is not None:
            m.status_list = [GoalStatus(status=status)]
        self.status_pub.publish(m)
        return time.monotonic()

    def set_job_state(self, name):
        self.job_pub.publish(String(data=name))
        return time.monotonic()

    def wait_idle_run(self, timeout=15.0):
        self.wait_run(timeout)
        self.wait_stack(lambda s: s["state"].startswith("healthy"), 3.0, "healthy stack")

    def wait_quiet_run(self, timeout=15.0):
        """Stand-in in RUN and stack healthy, whatever the output (no commander is needed for this)."""
        self.wait_for(lambda: self.current_state() == "run", timeout, "stand-in RUN")
        self.wait_stack(lambda s: s["state"].startswith("healthy"), 3.0, "healthy stack")

    def wait_stack_task(self, task, timeout, what, since=0.0):
        self.wait_stack(lambda s: s["task_id"] == task, timeout, what, since)

    def task_since(self, task, t):
        for ts, s in self.stacks_since(t):
            if s["task_id"] == task:
                return ts
        return None

    def timeline(self, t0):
        """Stack/stand-in/cmd_vel changes after t0 as compact (dt_ms, what) lists."""
        out = []
        last = None
        for ts, s in self.stacks_since(t0):
            key = (s["state"], s["task_id"], tuple(self.faulted_critical(s)), tuple(s["missing_critical"]))
            if key != last:
                out.append((round((ts - t0) * 1000), "stack", key[0], key[1], list(key[2]), list(key[3])))
                last = key
        for ts, s in self.states_since(t0):
            out.append((round((ts - t0) * 1000), "mcu", s["state"]))
        return sorted(out)

    def motion_gaps(self, t0, t1):
        """Intervals (start_s, end_s relative to t0) in which the stand-in output was zero, 50 ms resolution."""
        gaps, start = [], None
        for ts, nz in self.cmds_since(t0):
            if ts > t1:
                break
            if not nz and start is None:
                start = ts
            elif nz and start is not None:
                gaps.append((start - t0, ts - t0))
                start = None
        if start is not None:
            gaps.append((start - t0, t1 - t0))
        return gaps
