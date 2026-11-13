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
    def __init__(self, node, odometer=None, log=None, sim_now=None):
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient

        self.node = node
        self.log = log or (lambda m: print(m, flush=True))
        self.odo = odometer or Odometer(node)
        self.client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        self.records = []
        self._sim_now = sim_now

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

    def one_goal(self, idx, pose, should_stop):
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
            elif res.done():
                rec["result"] = STATUS.get(res.result().status, f"status{res.result().status}")
            else:
                rec["result"] = "error"
                rec["note"] = "no result after cancel"
        except Exception as e:  # keep the loop alive whatever happens
            rec["result"] = "error"
            rec["note"] = f"{type(e).__name__}: {e}"
        return self._finish(rec, t0w, t0s, d0)

    def _finish(self, rec, t0w, t0s, d0):
        rec["duration_wall_s"] = round(time.monotonic() - t0w, 2)
        rec["duration_sim_s"] = round(self.sim_now() - t0s, 2)
        rec["sim_start_s"] = round(t0s, 2)
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
            rec = self.one_goal(i, POSES[i % len(POSES)], stop)
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
