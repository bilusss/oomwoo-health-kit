"""Repeatable navigation workload: an endless loop over a fixed list of poses on the living_room map via NavigateToPose.

Poses were chosen once from the shipped map: free cells with >= 0.5 m clearance,
farthest-point sampled in the connected component of the spawn pose (-2.0, -0.5), ordered so consecutive goals are 2-3 m apart.
The order and yaws are fixed, so every session sees the same goal sequence; outcomes and timing still vary run to run
(M6_RECON section 5: 4-28 s for the same pose pair).

Per goal one record: result (succeeded | aborted | canceled | rejected | timeout | no_server | error), duration in sim and wall
seconds, path length from /odom (ground-truth odometry, owned by the Gazebo bridge) while the goal ran. A failed goal is logged
and the loop continues with the next pose. All times named *_sim_* are ROS (Gazebo) clock, *_wall_* are the host monotonic clock.

Standalone:  python3 workload.py [--duration-sim S] [--out goals.jsonl]     (stack must be running; sim-time node)
As a module: Workload(node).run(duration_sim_s, should_stop) -> list of records. The node must be spun by the caller and use
use_sim_time.
"""

import argparse
import json
import math
import sys
import threading
import time

# name, x, y, yaw (map frame, metres / radians)
POSES = [
    ("P1", -0.57, 1.49, 0.0),
    ("P2", 1.93, 1.44, -1.57),
    ("P3", 1.68, -0.21, -1.57),
    ("P4", 1.78, -1.86, 3.14),
    ("P5", -0.97, -1.86, 1.57),
    ("P6", -1.92, 0.94, 0.0),
    ("P7", 0.53, 0.89, -1.57),
    ("P0", -2.00, -0.50, 0.0),
]
# --workload mix (M6c): a fixed cycle of 13 goals that adds, to the plain loop, goals cancelled while driving (cancel after N sim s)
# a goal outside the map (the planner rejects it at once, 0.01 s: measured, it does NOT trigger BT recoveries) and an explicit
# recovery probe "R": the behavior_server actions spin 1.57 rad, back up 0.3 m, wait 3 s sent directly while the robot is idle
# (same command path as BT recoveries: behavior_server -> cmd_vel_nav -> smoother -> collision_monitor -> cmd_vel; NOT BT-triggered). (pose index into POSES or "UNREACH", cancel_after_sim_s or None)
UNREACH = ("UNREACH", 9.0, 9.0, 0.0)
MIX_CYCLE = [(0, None), (1, None), (2, 4.0), (3, None), ("U", 35.0), ("R", None), (4, None), (5, None), (6, 1.0), (7, None), (0, None),
             (1, 6.0), (2, None)]
GOAL_TIMEOUT_SIM_S = 150.0
SERVER_WAIT_S = 30.0

STATUS = {0: "unknown", 1: "accepted", 2: "executing", 3: "canceling", 4: "succeeded", 5: "canceled", 6: "aborted"}


class Odometer:
    """Path length from nav_msgs/Odometry; jumps over 0.5 m between samples (teleport/reset) are ignored."""

    def __init__(self, node):
        from nav_msgs.msg import Odometry

        self.lock = threading.Lock()
        self.total = 0.0
        self.last = None
        self.count = 0
        self.pos = None
        node.create_subscription(Odometry, "odom", self._cb, 50)

    def _cb(self, m):
        p = (m.pose.pose.position.x, m.pose.pose.position.y)
        with self.lock:
            if self.last is not None:
                d = math.hypot(p[0] - self.last[0], p[1] - self.last[1])
                if d < 0.5:
                    self.total += d
            self.last = self.pos = p
            self.count += 1

    def distance(self):
        with self.lock:
            return self.total


def _yaw_to_quat(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class Workload:
    def __init__(self, node, odometer=None, log=None, sim_now=None, mix=False):
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient

        self.node = node
        self.log = log or (lambda m: print(m, flush=True))
        self.odo = odometer or Odometer(node)
        self.client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        self.records = []
        self._sim_now = sim_now
        self.mix = mix

    def sim_now(self):
        if self._sim_now:
            return self._sim_now()
        return self.node.get_clock().now().nanoseconds / 1e9

    def wait_clock(self, timeout=30.0):
        """The node clock reads 0 until the first /clock message arrives; wait for it (and for it to advance)."""
        end = time.monotonic() + timeout
        first = None
        while time.monotonic() < end:
            t = self.sim_now()
            if t > 0:
                first = t if first is None else first
                if t > first:
                    return True
            time.sleep(0.1)
        return False

    def wait_server(self, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.client.wait_for_server(timeout_sec=0.5):
                return True
        return False

    def _goal_msg(self, x, y, yaw):
        from nav2_msgs.action import NavigateToPose

        g = NavigateToPose.Goal()
        g.pose.header.frame_id = "map"
        # stamp 0 = "latest transform" (the node may run on wall time while TF is on sim time)
        g.pose.pose.position.x, g.pose.pose.position.y = x, y
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = _yaw_to_quat(yaw)
        return g

    def one_goal(self, idx, pose, should_stop, cancel_after=None):
        """Send one goal and wait for the result. Never raises; returns the record."""
        name, x, y, yaw = pose
        rec = {"idx": idx, "pose": name, "x": x, "y": y, "yaw": yaw, "result": "error"}
        t0w, t0s, d0 = time.monotonic(), self.sim_now(), self.odo.distance()
        try:
            if not self.wait_server(SERVER_WAIT_S):
                rec["result"] = "no_server"
                return self._finish(rec, t0w, t0s, d0)
            sent = self.client.send_goal_async(self._goal_msg(x, y, yaw))
            ev = threading.Event()
            sent.add_done_callback(lambda _f: ev.set())
            while not ev.wait(0.2):
                if should_stop() or self.sim_now() - t0s > 30.0:
                    rec["result"] = "rejected" if not should_stop() else "canceled"
                    rec["note"] = "no goal response"
                    return self._finish(rec, t0w, t0s, d0)
            handle = sent.result()
            if not handle.accepted:
                rec["result"] = "rejected"
                return self._finish(rec, t0w, t0s, d0)
            res = handle.get_result_async()
            done = threading.Event()
            res.add_done_callback(lambda _f: done.set())
            cancelled_by = None
            while not done.wait(0.2):
                if should_stop():
                    cancelled_by = "session_end"
                elif cancel_after is not None and self.sim_now() - t0s > cancel_after:
                    cancelled_by = "test_cancel"
                elif self.sim_now() - t0s > GOAL_TIMEOUT_SIM_S:
                    cancelled_by = "timeout"
                if cancelled_by:
                    cf = handle.cancel_goal_async()
                    ce = threading.Event()
                    cf.add_done_callback(lambda _f: ce.set())
                    ce.wait(10.0)
                    done.wait(10.0)
                    break
            if cancelled_by == "timeout":
                rec["result"] = "timeout"
            elif cancelled_by == "session_end":
                rec["result"] = "canceled"
                rec["note"] = "session ended"
            elif cancelled_by == "test_cancel":
                rec["result"] = "canceled"
                rec["note"] = f"test cancel after {cancel_after} sim s" + (" (unplannable goal)" if pose[0] == "UNREACH" else "")
            elif res.done():
                rec["result"] = STATUS.get(res.result().status, f"status{res.result().status}")
            else:
                rec["result"] = "error"
                rec["note"] = "no result after cancel"
        except Exception as e:  # keep the loop alive whatever happens
            rec["result"] = "error"
            rec["note"] = f"{type(e).__name__}: {e}"
        return self._finish(rec, t0w, t0s, d0)

    def recovery_probe(self, idx, should_stop):
        """spin, back up, wait via the behavior_server action servers; one record like a goal."""
        from builtin_interfaces.msg import Duration
        from geometry_msgs.msg import Point
        from nav2_msgs.action import BackUp, Spin, Wait
        from rclpy.action import ActionClient

        rec = {"idx": idx, "pose": "RECOVERY", "result": "succeeded", "steps": []}
        t0w, t0s, d0 = time.monotonic(), self.sim_now(), self.odo.distance()
        try:
            for name, cls, goal in (("spin", Spin, Spin.Goal()), ("backup", BackUp, BackUp.Goal()), ("wait", Wait, Wait.Goal())):
                if name == "spin":
                    goal.target_yaw = 1.57
                elif name == "backup":
                    goal.target = Point(x=0.3)
                    goal.speed = 0.1
                    goal.time_allowance = Duration(sec=15)
                else:
                    goal.time = Duration(sec=3)
                cl = ActionClient(self.node, cls, name)
                if should_stop() or not cl.wait_for_server(timeout_sec=5.0):
                    rec["steps"].append(f"{name}: no server/stop")
                    rec["result"] = "error"
                    continue
                ev = threading.Event()
                f = cl.send_goal_async(goal)
                f.add_done_callback(lambda _f: ev.set())
                if not ev.wait(10.0) or not f.result().accepted:
                    rec["steps"].append(f"{name}: not accepted")
                    rec["result"] = "error"
                    continue
                rf = f.result().get_result_async()
                ev2 = threading.Event()
                rf.add_done_callback(lambda _f: ev2.set())
                ok = ev2.wait(30.0)
                rec["steps"].append(f"{name}: " + (STATUS.get(rf.result().status, "?") if ok else "timeout"))
                cl.destroy()
        except Exception as e:  # keep the loop alive
            rec["result"], rec["note"] = "error", f"{type(e).__name__}: {e}"
        return self._finish(rec, t0w, t0s, d0)

    def _finish(self, rec, t0w, t0s, d0):
        rec["duration_wall_s"] = round(time.monotonic() - t0w, 2)
        rec["duration_sim_s"] = round(self.sim_now() - t0s, 2)
        rec["sim_start_s"] = round(t0s, 2)
        rec["t0_mono"], rec["t1_mono"] = t0w, time.monotonic()  # host monotonic clock, shared with the recorder process
        rec["distance_odom_m"] = round(self.odo.distance() - d0, 3)
        self.records.append(rec)
        self.log("goal " + json.dumps(rec))
        return rec

    def run(self, duration_sim_s=None, should_stop=lambda: False, on_goal=None):
        """Loop over POSES until duration_sim_s of sim time have passed (None = until should_stop)."""
        if not self.wait_clock():
            self.log("workload: no advancing /clock")
            return self.records
        t0 = self.sim_now()
        i = 0
        while not should_stop():
            if duration_sim_s is not None and self.sim_now() - t0 >= duration_sim_s:
                break
            stop = (lambda: should_stop() or (duration_sim_s is not None and self.sim_now() - t0 >= duration_sim_s))
            if self.mix:
                pi, cancel_after = MIX_CYCLE[i % len(MIX_CYCLE)]
                if pi == "R":
                    self.recovery_probe(i, stop)
                    i += 1
                    continue
                pose = UNREACH if pi == "U" else POSES[pi]
            else:
                pose, cancel_after = POSES[i % len(POSES)], None
            rec = self.one_goal(i, pose, stop, cancel_after)
            if on_goal:
                on_goal(rec)
            if rec["result"] in ("error", "no_server", "rejected"):
                time.sleep(2.0)  # do not spin on a broken server
            i += 1
        return self.records


def main():
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter

    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--duration-sim", type=float, default=None, help="sim seconds to run (default: until SIGINT)")
    ap.add_argument("--out", default=None, help="append one JSON line per goal here")
    a = ap.parse_args()
    rclpy.init()
    node = Node("ohk_workload", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    w = Workload(node)
    out = open(a.out, "a") if a.out else None
    try:
        w.run(a.duration_sim, on_goal=(lambda r: (out.write(json.dumps(r) + "\n"), out.flush())) if out else None)
    except KeyboardInterrupt:
        pass
    ok = sum(1 for r in w.records if r["result"] == "succeeded")
    print(f"goals {ok}/{len(w.records)} succeeded, distance {w.odo.distance():.2f} m", file=sys.stderr)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
