"""M7 workloads for bench/fullstack/run.py: coverage, docking and BT-triggered recoveries.

Each class has the interface the worker uses for the M6 `Workload`: `.run(duration_sim_s, should_stop, on_goal)`, `.records`
(dicts with result, duration_*, distance_odom_m, t0_mono/t1_mono, idx, pose), `.odo`, `.sim_now()`, `.ready()`. They inherit the goal
sender and the record bookkeeping from workload.Workload.

  CoverageJob  runs the upstream coverage_planner (oomwoo_coverage, as in oomwoo_sim_support coverage_regression.launch.py) as a
               subprocess started after the health loop is armed; one record per planner process (restarted after it exits).
  DockJob      cycles through DOCK_STARTS: teleports the robot to a start pose (HARNESS ONLY, gz set_pose), enables dock_drive
               through its own ~/enable topic, waits for DONE (docked) or IDLE (gave up); one record per attempt.
  RecoveryJob  NavigateToPose goals with a wall that the map does not know spawned across the path while the robot drives, so
               that bt_navigator's recovery branch (clear costmaps, spin, wait, backup) runs; nothing sends recovery actions
               directly. One record per goal with the BT recovery actions that started during it.
"""

import json
import math
import os
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import ohk_bench as b  # noqa: E402
import tasks as tk  # noqa: E402
from workload import POSES, Workload  # noqa: E402

COVERAGE_PARAMS = {"cleaning_radius": 0.20, "robot_radius": 0.18, "coverage_target": 0.90, "row_overlap": 0.0, "max_retries": 1,
                   "contact_aware_escape": True, "row_substep_m": 0.4, "long_axis_sweep": True, "v_cruise": 0.35,
                   "min_transit_len": 0.7, "use_sim_time": True}


def planner_cmd(executor, prefix=None):
    """argv of the upstream coverage_planner with the parameters and remaps of coverage_regression.launch.py (the executable is
    started directly, not through `ros2 run`, so a signal reaches the planner itself)."""
    if prefix is None:
        from ament_index_python.packages import get_package_prefix
        prefix = get_package_prefix("oomwoo_coverage")
    argv = [os.path.join(prefix, "lib", "oomwoo_coverage", "coverage_planner"), "--ros-args"]
    for k, v in {**COVERAGE_PARAMS, "executor": executor}.items():
        argv += ["-p", f"{k}:={str(v).lower() if isinstance(v, bool) else v}"]
    for a, c in (("map", "/map"), ("coverage_ratio", "/coverage_meter/ratio"), ("covered_grid", "/coverage_meter/covered_grid"),
                 ("navigate_to_pose", "/navigate_to_pose")):
        argv += ["-r", f"{a}:={c}"]
    return argv


class CoverageJob(Workload):
    def __init__(self, node, executor, log_dir="/tmp", tally=None, restart=True, **kw):
        super().__init__(node, **kw)
        self.restart = restart
        from std_msgs.msg import Float32
        self.executor = executor
        self.log_dir = log_dir
        self.proc = None
        self.proc_log = None
        self.ratio = None
        self.tally = tally
        node.create_subscription(Float32, "/coverage_meter/ratio", lambda m: setattr(self, "ratio", float(m.data)), 10)

    def ready(self):
        return self.client.server_is_ready()

    def start_planner(self, n):
        path = os.path.join(self.log_dir, f"coverage_planner_{n}.log")
        self.proc_log = path
        self.proc = subprocess.Popen(planner_cmd(self.executor), stdout=open(path, "w"), stderr=subprocess.STDOUT,
                                     start_new_session=True, preexec_fn=b.die_with_parent)
        self.log(f"coverage_planner ({self.executor}) pid {self.proc.pid} log {path}")

    def stop_planner(self):
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(10)
            except subprocess.TimeoutExpired:
                self.proc.kill()

    def run(self, duration_sim_s=None, should_stop=lambda: False, on_goal=None):
        if not self.wait_clock():
            self.log("coverage: no advancing /clock")
            return self.records
        t0 = self.sim_now()
        n = 0
        while not should_stop() and (duration_sim_s is None or self.sim_now() - t0 < duration_sim_s):
            n += 1
            rec = {"idx": n - 1, "pose": "COVERAGE", "executor": self.executor, "result": "error"}
            t0w, t0s, d0 = time.monotonic(), self.sim_now(), self.odo.distance()
            self.start_planner(n)
            while not should_stop() and self.proc.poll() is None and (duration_sim_s is None or self.sim_now() - t0 < duration_sim_s):
                time.sleep(0.5)
            if self.proc.poll() is None:
                rec["result"] = "canceled"
                rec["note"] = "session ended"
                self.stop_planner()
            else:
                rec["result"] = "succeeded" if self.proc.returncode == 0 else "error"
                rec["note"] = f"planner exited with {self.proc.returncode}"
            try:
                rec["planner"] = tk.parse_planner_log(open(self.proc_log, errors="replace").read().splitlines())
            except OSError:
                rec["planner"] = None
            rec["coverage_ratio_meter"] = self.ratio
            self._finish(rec, t0w, t0s, d0)
            if on_goal:
                on_goal(rec)
            if rec["result"] != "canceled":
                if not self.restart:   # fault trials: a planner that was killed must stay dead
                    while not should_stop():
                        time.sleep(0.5)
                    break
                time.sleep(3.0)   # a planner that exits at once must not spin the loop
        return self.records


class DockJob(Workload):
    def __init__(self, node, model="oomwoo_one", **kw):
        super().__init__(node, **kw)
        from std_msgs.msg import Bool, String
        self.model = model
        self.state = None
        self.states = []
        self.lock = threading.Lock()
        self.enable_pub = node.create_publisher(Bool, "/dock_drive/enable", 10)
        node.create_subscription(String, "/dock_drive/state", self._on_state, 10)
        self._Bool = Bool
        self.node = node
        self.gz = Gz()

    def _on_state(self, m):
        with self.lock:
            self.state = m.data
            self.states.append((time.monotonic(), m.data))

    def ready(self):
        return self.node.count_subscribers("/dock_drive/enable") > 0 and self.node.count_publishers("/dock_detector/dock_pose") > 0

    def _enable(self, on):
        self.enable_pub.publish(self._Bool(data=bool(on)))

    def teleport(self, pose):
        _name, x, y, yaw = pose
        return self.gz.set_pose(self.model, x, y, 0.06, yaw)

    def warm(self):
        """First gz-transport service call includes discovery (about 0.1 s stall of the sim); do it before the measured window."""
        return self.teleport(tk.DOCK_STARTS[0])

    def attempt(self, idx, pose, should_stop):
        rec = {"idx": idx, "pose": "DOCK:" + pose[0], "result": "error"}
        self._enable(False)
        time.sleep(0.5)
        rec["harness_actions"] = [{"what": "set_pose", "epoch": time.time()}]
        rec["teleport"] = self.teleport(pose)
        rec["harness_actions"].append({"what": "set_pose_done", "epoch": time.time()})
        time.sleep(2.0)   # a fresh scan from the new pose, odometry settled
        t0w, t0s, d0 = time.monotonic(), self.sim_now(), self.odo.distance()
        with self.lock:
            self.states.clear()
        self._enable(True)
        res = None
        while not should_stop():
            with self.lock:
                terminal = [s for _t, s in self.states if s in tk.DOCK_TERMINAL]
            if terminal:
                res = tk.DOCK_TERMINAL[terminal[0]]
                break
            if self.sim_now() - t0s > tk.DOCK_ATTEMPT_TIMEOUT_SIM_S:
                res = "timeout"
                break
            time.sleep(0.2)
        with self.lock:
            rec["states"] = [s for _t, s in self.states]
        rec["result"] = res or "canceled"
        if res is None:
            rec["note"] = "session ended"
        self._enable(False)
        return self._finish(rec, t0w, t0s, d0)

    def run(self, duration_sim_s=None, should_stop=lambda: False, on_goal=None):
        if not self.wait_clock():
            self.log("dock: no advancing /clock")
            return self.records
        t0 = self.sim_now()
        i = 0
        while not should_stop() and (duration_sim_s is None or self.sim_now() - t0 < duration_sim_s):
            stop = lambda: should_stop() or (duration_sim_s is not None and self.sim_now() - t0 >= duration_sim_s)  # noqa: E731
            rec = self.attempt(i, tk.DOCK_STARTS[i % len(tk.DOCK_STARTS)], stop)
            if on_goal:
                on_goal(rec)
            i += 1
            time.sleep(1.0)
        return self.records


class RecoveryJob(Workload):
    WALL = "ohk_wall"

    def __init__(self, node, tally=None, tmp="/tmp", **kw):
        super().__init__(node, **kw)
        self.tally = tally
        self.gz = Gz()
        self.sdf = os.path.join(tmp, "ohk_wall.sdf")
        with open(self.sdf, "w") as f:
            f.write(tk.wall_sdf(self.WALL))

    def ready(self):
        return self.client.server_is_ready()

    def remove_wall(self):
        return self.gz.remove(self.WALL)

    def spawn_wall(self, x, y, yaw):
        return self.gz.create_box(self.WALL, self.sdf, x, y, yaw)

    def warm(self):
        self.spawn_wall(0.0, 0.0, 0.0)   # discovery of the create and remove services (the wall sits at the origin for an instant)
        return self.remove_wall()

    def scenario(self, idx, goal_i, wall, should_stop):
        pose = POSES[goal_i]
        start = self.odo.pos
        far = start is not None and math.hypot(pose[1] - start[0], pose[2] - start[1]) >= tk.MIN_GOAL_DISTANCE_M
        info = {"wall": wall if (wall and far) else (None if not wall else "skipped_goal_too_close")}
        out = {}

        def walls():
            t0 = self.sim_now()
            while self.sim_now() - t0 < tk.WALL_DELAY_SIM_S and not should_stop() and not out.get("done"):
                time.sleep(0.1)
            if out.get("done") or should_stop() or info["wall"] not in ("persistent", "transient"):
                return
            p = self.odo.pos
            x, y, yaw = tk.wall_ahead(p, (pose[1], pose[2]))
            out.setdefault("harness_actions", []).append({"what": "create_wall", "epoch": time.time()})
            out["spawn_reply"] = self.spawn_wall(x, y, yaw)
            out["wall_at"] = [round(x, 2), round(y, 2)]
            out["wall_placed_mono"] = time.monotonic()
            if info["wall"] == "transient":
                t1 = self.sim_now()
                while self.sim_now() - t1 < tk.TRANSIENT_AFTER_SIM_S and not should_stop() and not out.get("done"):
                    time.sleep(0.1)
                out.setdefault("harness_actions", []).append({"what": "remove_wall", "epoch": time.time()})
                out["remove_reply"] = self.remove_wall()
                out["wall_removed_mono"] = time.monotonic()

        th = threading.Thread(target=walls, daemon=True)
        th.start()
        rec = self.one_goal(idx, pose, should_stop, cancel_after=tk.RECOVERY_GOAL_MAX_SIM_S)
        out["done"] = True
        th.join(timeout=15)
        if "wall_placed_mono" in out and "wall_removed_mono" not in out:
            out.setdefault("harness_actions", []).append({"what": "remove_wall", "epoch": time.time()})
            out["remove_reply"] = self.remove_wall()
        rec["scenario"] = {**info, **{k: v for k, v in out.items() if k != "done"}}
        rec["harness_actions"] = out.get("harness_actions", [])
        # BT recovery actions that started during the goal (from the status topics, bench/fullstack/run.py)
        if self.tally is not None:
            rec["bt_recoveries"] = self.tally.started_between(rec["t0_mono"], rec["t1_mono"])
        return rec

    def run(self, duration_sim_s=None, should_stop=lambda: False, on_goal=None):
        if not self.wait_clock():
            self.log("recovery: no advancing /clock")
            return self.records
        self.remove_wall()   # a wall left over from an aborted earlier run
        t0 = self.sim_now()
        i = 0
        while not should_stop() and (duration_sim_s is None or self.sim_now() - t0 < duration_sim_s):
            stop = lambda: should_stop() or (duration_sim_s is not None and self.sim_now() - t0 >= duration_sim_s)  # noqa: E731
            gi, wall = tk.RECOVERY_CYCLE[i % len(tk.RECOVERY_CYCLE)]
            rec = self.scenario(i, gi, wall, stop)
            if on_goal:
                on_goal(rec)
            if rec["result"] in ("error", "no_server", "rejected"):
                time.sleep(2.0)
            else:
                time.sleep(1.0)
            i += 1
        self.remove_wall()
        return self.records


class RecoveryProbeJob(Workload):
    """M8 workload for behavior_server fault trials: NavigateToPose to an open pose, then LONG direct spin and backup actions (no BT, no
    wall), so that a spin at its nominal 1 rad/s or a backup at 0.15 m/s lasts several seconds and the fault trial can be signalled
    inside it. Same command path as BT recoveries (behavior_server -> /cmd_vel_nav -> smoother -> collision monitor -> /cmd_vel). One
    record per goal and per probe. Spin 3.14 rad at the behavior_server's 1.0 rad/s limit takes about 3.3 s; backup 0.45 m at 0.15 m/s 3 s."""
    PROBE = {"spin_rad": 3.14, "backup_m": 0.45, "backup_mps": 0.15, "steps": ("spin",)}
    BACKUP = {**PROBE, "steps": ("backup",)}

    def ready(self):
        return self.client.server_is_ready()

    def run(self, duration_sim_s=None, should_stop=lambda: False, on_goal=None):
        if not self.wait_clock():
            self.log("recovery_probe: no advancing /clock")
            return self.records
        t0 = self.sim_now()
        i = 0
        while not should_stop() and (duration_sim_s is None or self.sim_now() - t0 < duration_sim_s):
            stop = lambda: should_stop() or (duration_sim_s is not None and self.sim_now() - t0 >= duration_sim_s)  # noqa: E731
            rec = self.one_goal(i, POSES[1 + i % 6], stop)     # P2..P6, P7: open poses
            if on_goal:
                on_goal(rec)
            i += 1
            for spec in (self.PROBE, self.BACKUP):
                if stop():
                    break
                rec = self.recovery_probe(i, stop, spec)
                if on_goal:
                    on_goal(rec)
                i += 1
                time.sleep(1.0)
        return self.records


class Gz:
    """Persistent gz-transport client (python bindings shipped with ros_gz): the `gz` CLI costs ~0.3 s of CPU and ~2 s of start-up per call,
    which stalled the /odom and /cmd_vel streams of the stack under test (M7 smoke runs); this does not start a process per call."""

    def __init__(self, world="default"):
        from gz.transport13 import Node
        self.node = Node()
        self.world = world

    def _call(self, service, req, rep_cls, timeout_ms=5000):
        ok, rep = self.node.request(service, req, type(req), rep_cls, timeout_ms)
        return bool(ok and getattr(rep, "data", True))

    def set_pose(self, model, x, y, z, yaw):
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.pose_pb2 import Pose
        p = Pose()
        p.name = model
        p.position.x, p.position.y, p.position.z = x, y, z
        p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return self._call(f"/world/{self.world}/set_pose", p, Boolean)

    def create_box(self, name, sdf_path, x, y, yaw, z=0.5):
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.entity_factory_pb2 import EntityFactory
        r = EntityFactory()
        r.sdf_filename, r.name = sdf_path, name
        r.pose.position.x, r.pose.position.y, r.pose.position.z = x, y, z
        r.pose.orientation.z, r.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return self._call(f"/world/{self.world}/create", r, Boolean)

    def remove(self, name):
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.entity_pb2 import Entity
        e = Entity()
        e.name, e.type = name, Entity.MODEL
        return self._call(f"/world/{self.world}/remove", e, Boolean)
