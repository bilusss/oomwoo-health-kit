"""Shared launch description + probe node for the fault-injection tests."""

import json
import os
import signal
import threading
import time
import unittest

import launch
import launch_testing.actions
import rclpy
from geometry_msgs.msg import Twist
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node as RclNode
from std_msgs.msg import String

from oomwoo_health_kit.qos import latched_state_qos, live_heartbeat_qos, mcu_heartbeat_qos

# Isolate every test process group on its own DDS domain, localhost only.
os.environ["ROS_DOMAIN_ID"] = str(20 + os.getpid() % 200)
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"

MAX_AGE = 0.15
SOFT_STOP_S = 0.15  # stand-in default (experimental simulation setting)
TICK_S = 0.01  # stand-in update timer
# Latency model, kill/hang/stall -> first zero Twist: aggregator max_age + stand-in soft stop + one tick + margin.
MARGIN_S = 0.10
FAULT_TO_STOP_MAX_S = MAX_AGE + SOFT_STOP_S + TICK_S + MARGIN_S
# Aggregator killed: heartbeats just stop, so soft stop + one tick + margin.
AGG_KILLED_STOP_MAX_S = SOFT_STOP_S + TICK_S + 0.05
AGG_KILLED_RESET_TIMEOUT_S = 2.0  # explicit short value: the default (300 s) would make test f take minutes
RESULTS = os.environ.get("OHK_RESULTS", "/tmp/ohk_results.json")
ACTIONS = {}


def build_description(
    critical=("critical_a", "critical_b"),
    advisory=("advisory_a",),
    started=None,
    roster=True,
    roster_critical=None,
    timer_beating=(),
    mcu_params=None,
    sim_time=False,
):
    """`started`: component ids to launch (default all); `roster_critical`: ids in roster;
    `mcu_params`: extra stand-in parameters (e.g. a short cpu_reset_timeout_s);
    `sim_time`: use_sim_time on every node (the test must then publish /clock)."""
    ACTIONS.clear()
    started = list(critical) + list(advisory) if started is None else list(started)
    roster_critical = list(critical) if roster_critical is None else list(roster_critical)
    ld = launch.LaunchDescription()

    def add(name, **kw):
        if sim_time:
            kw["parameters"] = [*kw.get("parameters", []), {"use_sim_time": True}]
        ACTIONS[name] = Node(name=name, **kw)
        ld.add_action(ACTIONS[name])

    add("health_monitor", package="oomwoo_health_monitor", executable="health_monitor_node")
    add("mcu_standin", package="oomwoo_health_kit", executable="mcu_standin_node", parameters=[dict(mcu_params or {})])
    if roster:
        add(
            "roster_publisher",
            package="oomwoo_health_kit",
            executable="roster_publisher_node",
            parameters=[
                {
                    "task_id": "ohk_test",
                    "critical_ids": roster_critical,
                    "advisory_ids": list(advisory),
                    "max_age_sec": ParameterValue(MAX_AGE, value_type=float),
                }
            ],
        )
    for cid in started:
        add(
            cid,
            package="oomwoo_health_kit",
            executable="fake_component_node",
            parameters=[{"component_id": cid, "rate_hz": 20.0, "beat_from_timer": cid in timer_beating}],
        )
    ld.add_action(launch_testing.actions.ReadyToTest())
    return ld


class Probe(RclNode):
    """Publishes a constant non-zero Twist at 20 Hz; records cmd_vel and mcu_state."""

    def __init__(self):
        super().__init__("ohk_probe")
        self.lock = threading.Lock()
        self.cmds = []  # (monotonic_t, nonzero: bool)
        self.states = []  # (monotonic_t, dict)
        self.stacks = []  # (monotonic_t, dict) from oomwoo/health/stack
        self.comp_details = []  # (monotonic_t, component_id, detail)
        self.comp_stamps = []  # (monotonic_t, component_id, stamp_sec)
        self.mcu_hbs = []  # (monotonic_t, dict) aggregate MCU heartbeats from the aggregator
        self.feed = True  # False: the probe publishes no command (a test's own commander feeds cmd_vel_in)
        self._pub = self.create_publisher(Twist, "cmd_vel_in", 10)
        self.create_subscription(Twist, "cmd_vel", self._on_cmd, 10)
        self.create_subscription(String, "oomwoo/health/mcu_state", self._on_state, latched_state_qos())
        self.create_subscription(String, "oomwoo/health/stack", self._on_stack, latched_state_qos())
        self.create_subscription(String, "oomwoo/health/component", self._on_comp, live_heartbeat_qos())
        self.create_subscription(String, "oomwoo/health/mcu_heartbeat", self._on_mcu_hb, mcu_heartbeat_qos())
        self.create_timer(0.05, self._tick)

    def _on_mcu_hb(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            d = {}
        with self.lock:
            self.mcu_hbs.append((time.monotonic(), d))

    def _tick(self):
        if not self.feed:
            return
        m = Twist()
        m.linear.x = 0.2
        self._pub.publish(m)

    def _on_cmd(self, m):
        nz = any((m.linear.x, m.linear.y, m.linear.z, m.angular.x, m.angular.y, m.angular.z))
        with self.lock:
            self.cmds.append((time.monotonic(), nz))

    def _on_stack(self, m):
        with self.lock:
            self.stacks.append((time.monotonic(), json.loads(m.data)))

    def _on_comp(self, m):
        d = json.loads(m.data)
        with self.lock:
            self.comp_details.append((time.monotonic(), d["component_id"], d.get("detail", "")))
            self.comp_stamps.append((time.monotonic(), d["component_id"], d.get("stamp_sec")))

    def _on_state(self, m):
        with self.lock:
            self.states.append((time.monotonic(), json.loads(m.data)))


class OhkCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()
        cls.ex = SingleThreadedExecutor()
        cls.ex.add_node(cls.probe)
        cls.thread = threading.Thread(target=cls.ex.spin, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        for a in ACTIONS.values():  # never leave a SIGSTOPped process behind
            try:
                os.kill(a.process_details["pid"], signal.SIGCONT)
            except Exception:
                pass
        cls.ex.shutdown()
        cls.thread.join(2)
        cls.probe.destroy_node()
        rclpy.shutdown()

    # --- observation helpers
    def wait_for(self, pred, timeout, what):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if pred():
                return
            time.sleep(0.01)
        self.fail(f"timeout ({timeout}s) waiting for {what}")

    def cmds_since(self, t):
        with self.probe.lock:
            return [c for c in self.probe.cmds if c[0] >= t]

    def states_since(self, t):
        with self.probe.lock:
            return [s for s in self.probe.states if s[0] >= t]

    def current_state(self):
        with self.probe.lock:
            return self.probe.states[-1][1]["state"] if self.probe.states else None

    def wait_run(self, timeout=15.0):
        def ok():
            c = self.cmds_since(time.monotonic() - 0.3)
            return self.current_state() == "run" and c and c[-1][1]

        self.wait_for(ok, timeout, "RUN with non-zero cmd_vel")

    def pid(self, name):
        def started():
            a = ACTIONS.get(name)
            d = a.process_details if a is not None else None
            return bool(d) and "pid" in d

        self.wait_for(started, 10, f"{name} start")
        return ACTIONS[name].process_details["pid"]

    def mcu_hbs_since(self, t):
        with self.probe.lock:
            return [h for h in self.probe.mcu_hbs if h[0] >= t]

    def stacks_since(self, t):
        with self.probe.lock:
            return [s for s in self.probe.stacks if s[0] >= t]

    def last_stack(self):
        with self.probe.lock:
            return self.probe.stacks[-1][1] if self.probe.stacks else None

    def wait_stack(self, pred, timeout, what, since=0.0):
        self.wait_for(lambda: any(pred(s) for _, s in self.stacks_since(since)), timeout, what)

    @staticmethod
    def faulted_critical(stack):
        """Critical components the aggregator blames: stale + unhealthy (missing separate)."""
        return sorted(stack["stale_critical"] + stack["unhealthy_critical"])

    def assert_blames(self, name, since):
        """Stack names exactly `name` (and nothing missing) as the critical fault."""
        self.wait_stack(
            lambda s: s["state"] == "fault" and self.faulted_critical(s) == [name] and not s["missing_critical"],
            2.0,
            f"stack blaming exactly {name}",
            since,
        )

    def signal_and_time_stop(self, name, sig, timeout=3.0):
        """Send `sig`; return latency (s) to the first zero Twist observed afterwards."""
        pid = self.pid(name)
        t0 = time.monotonic()
        os.kill(pid, sig)
        self.wait_for(lambda: any(not nz for _, nz in self.cmds_since(t0)), timeout, "zero cmd_vel")
        first = next(t for t, nz in self.cmds_since(t0) if not nz)
        return t0, first - t0

    def record(self, scenario, **fields):
        entry = {"scenario": scenario, **fields}
        data = []
        if os.path.exists(RESULTS):
            try:
                data = json.load(open(RESULTS))
            except ValueError:
                data = []
        data.append(entry)
        json.dump(data, open(RESULTS, "w"), indent=1)
