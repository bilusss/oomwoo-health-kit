"""Shared helpers for the M3 benchmarks. Runs inside the ohk-dev container (ROS sourced).

Not part of `colcon test`. Spawns the real nodes as separate processes, observes them
with a probe node, and tears everything down in `finally`.
"""

import json
import os
import signal
import statistics
import subprocess
import sys
import threading
import time

from ament_index_python.packages import get_package_prefix

LOAD_PROCS = 12  # this PC: 6 cores / 12 threads; never go above 12


def exe(pkg, name):
    return os.path.join(get_package_prefix(pkg), "lib", pkg, name)


class Procs:
    """Child processes, each in its own session so the whole group can be killed."""

    def __init__(self, domain):
        self.env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST",
                        PYTHONUNBUFFERED="1")
        self.by_name = {}

    def start(self, name, argv, extra_env=None, quiet=True):
        out = subprocess.DEVNULL if quiet else None
        env = dict(self.env, **(extra_env or {}))
        self.by_name[name] = subprocess.Popen(argv, env=env, start_new_session=True, stdout=out, stderr=out)
        return self.by_name[name]

    def pid(self, name):
        return self.by_name[name].pid

    def kill(self, name, sig=signal.SIGKILL):
        p = self.by_name[name]
        try:
            os.killpg(p.pid, sig)
        except ProcessLookupError:
            pass

    def stop_all(self):
        for n in list(self.by_name):
            self.kill(n, signal.SIGINT)
        end = time.monotonic() + 4
        for p in self.by_name.values():
            try:
                p.wait(max(0.1, end - time.monotonic()))
            except subprocess.TimeoutExpired:
                pass
        for n in list(self.by_name):
            self.kill(n, signal.SIGKILL)
        for p in self.by_name.values():
            try:
                p.wait(2)
            except subprocess.TimeoutExpired:
                pass


class CpuLoad:
    """N busy-loop python processes (a pure-Python `while True: pass`)."""

    def __init__(self, n):
        assert 0 <= n <= LOAD_PROCS
        self.n = n
        self.procs = []

    def __enter__(self):
        for _ in range(self.n):
            self.procs.append(subprocess.Popen([sys.executable, "-c", "while True: pass"], start_new_session=True))
        return self

    def __exit__(self, *a):
        for p in self.procs:
            p.kill()
        for p in self.procs:
            p.wait()


def kit_nodes(procs, max_age, rate, clock_source="steady", sim_time=False, critical=("critical_a", "critical_b"),
              advisory=("advisory_a",), mcu_extra=()):
    """Aggregator + MCU stand-in + roster + fake components (the M2 happy-path loop)."""
    st = ["-p", f"use_sim_time:={'true' if sim_time else 'false'}"]
    procs.start("health_monitor", [exe("oomwoo_health_monitor", "health_monitor_node"), "--ros-args", *st])
    procs.start("mcu_standin", [exe("oomwoo_health_kit", "mcu_standin_node"), "--ros-args", *st,
                                "-p", f"clock_source:={clock_source}", *mcu_extra])
    procs.start("roster_publisher", [exe("oomwoo_health_kit", "roster_publisher_node"), "--ros-args", *st,
                                     "-p", "task_id:=bench",
                                     "-p", f"critical_ids:=[{','.join(critical)}]",
                                     "-p", f"advisory_ids:=[{','.join(advisory)}]",
                                     "-p", f"max_age_sec:={max_age}"])
    for cid in list(critical) + list(advisory):
        start_component(procs, cid, rate, sim_time)


def start_component(procs, cid, rate, sim_time=False):
    procs.start(cid, [exe("oomwoo_health_kit", "fake_component_node"), "--ros-args",
                      "-r", f"__node:={cid}", "-p", f"component_id:={cid}", "-p", f"rate_hz:={float(rate)}",
                      "-p", f"use_sim_time:={'true' if sim_time else 'false'}"])


def make_probe(sim_time=False, cmd_speed=0.2, publish_cmd=True):
    """Build probe node class lazily (rclpy must be initialised by the caller)."""
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from std_msgs.msg import String

    from oomwoo_health_kit.qos import latched_state_qos, live_heartbeat_qos

    class Probe(Node):
        def __init__(self):
            super().__init__("bench_probe", parameter_overrides=[])
            self.lock = threading.Lock()
            self.cmds, self.states, self.stacks, self.hbs = [], [], [], []
            self._pub = self.create_publisher(Twist, "cmd_vel_in", 10)
            self.create_subscription(Twist, "cmd_vel", self._on_cmd, 10)
            self.create_subscription(String, "oomwoo/health/mcu_state", self._on_state, latched_state_qos())
            self.create_subscription(String, "oomwoo/health/stack", self._on_stack, latched_state_qos())
            self.create_subscription(String, "oomwoo/health/mcu_heartbeat", self._on_hb, live_heartbeat_qos())
            self.speed = cmd_speed
            if publish_cmd:
                self.create_timer(0.05, self._tick)

        def _tick(self):
            m = Twist()
            m.linear.x = self.speed
            self._pub.publish(m)

        def _on_cmd(self, m):
            with self.lock:
                self.cmds.append((time.monotonic(), m.linear.x != 0.0 or m.angular.z != 0.0))

        def _on_state(self, m):
            with self.lock:
                self.states.append((time.monotonic(), json.loads(m.data)))

        def _on_stack(self, m):
            with self.lock:
                self.stacks.append((time.monotonic(), json.loads(m.data)))

        def _on_hb(self, m):
            with self.lock:
                self.hbs.append(time.monotonic())

        def snapshot(self):
            with self.lock:
                return list(self.cmds), list(self.states), list(self.stacks), list(self.hbs)

        def state(self):
            with self.lock:
                return self.states[-1][1]["state"] if self.states else None

        def last_cmd(self):
            with self.lock:
                return self.cmds[-1] if self.cmds else None

    return Probe()


def spin_in_thread(node):
    from rclpy.executors import SingleThreadedExecutor

    ex = SingleThreadedExecutor()
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    return ex, th


def wait_for(pred, timeout, step=0.01):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(step)
    return False


def wait_run(probe, timeout):
    def ok():
        c = probe.last_cmd()
        return probe.state() == "run" and c is not None and c[1] and time.monotonic() - c[0] < 0.3

    return wait_for(ok, timeout)


def summarize(xs):
    if not xs:
        return {"n": 0}
    return {"n": len(xs), "min": min(xs), "median": statistics.median(xs), "max": max(xs)}
