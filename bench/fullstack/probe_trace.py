#!/usr/bin/env python3
"""Raw message trace for M8 recon (run inside an ohk-sim-<i> while a stack and a workload run).

  python3 probe_trace.py --seconds 600 --out trace.jsonl

Subscribes (receive time = host monotonic clock) to the command chain, the dock topics and the Nav2 action status topics and writes
one JSON line per message to --out, flushed as it goes:
  ["t", topic, lin_x, ang_z]                    Twist topics (/cmd_vel_nav, /cmd_vel_smoothed, /cmd_vel, /cmd_vel_gated)
  ["t", topic, "STATE"]                         /dock_drive/state (std_msgs/String; published on change only)
  ["t", topic, stamp_sec, x, y]                 /dock_detector/dock_pose
  ["t", topic, [[goal_id_hex, status], ...]]    <action>/_action/status for spin, backup, wait, drive_on_heading, navigate_to_pose
  ["t", "/odom", v, w]                          /odom twist (10 Hz sub-sampled)
Offline analysis of the trace is in bench/fullstack/m8_analysis.py. Nothing here changes the graph (volatile subscriptions only).
"""

import argparse
import json
import threading
import time

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

REL = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=200)
BE = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=200)
TWISTS = ("/cmd_vel_nav", "/cmd_vel_smoothed", "/cmd_vel", "/cmd_vel_gated")
ACTIONS = ("/spin", "/backup", "/wait", "/drive_on_heading", "/navigate_to_pose")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=600.0)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    rclpy.init()
    node = Node("ohk_probe_trace")
    lock = threading.Lock()
    f = open(a.out, "w")

    def w(row):
        with lock:
            f.write(json.dumps(row, separators=(",", ":")) + "\n")

    for t in TWISTS:
        node.create_subscription(Twist, t, lambda m, t=t: w([time.monotonic(), t, round(m.linear.x, 4), round(m.angular.z, 4)]), REL)
    node.create_subscription(String, "/dock_drive/state", lambda m: w([time.monotonic(), "/dock_drive/state", m.data]), REL)
    node.create_subscription(PoseStamped, "/dock_detector/dock_pose", lambda m: w(
        [time.monotonic(), "/dock_detector/dock_pose", m.header.stamp.sec + m.header.stamp.nanosec / 1e9,
         round(m.pose.position.x, 3), round(m.pose.position.y, 3)]), REL)
    last_odom = [0.0]

    def on_odom(m):
        t = time.monotonic()
        if t - last_odom[0] >= 0.1:
            last_odom[0] = t
            w([t, "/odom", round(m.twist.twist.linear.x, 4), round(m.twist.twist.angular.z, 4)])
    node.create_subscription(Odometry, "/odom", on_odom, BE)
    for act in ACTIONS:
        topic = act + "/_action/status"
        node.create_subscription(GoalStatusArray, topic, lambda m, topic=topic: w(
            [time.monotonic(), topic, [[bytes(s.goal_info.goal_id.uuid).hex()[:8], s.status] for s in m.status_list]]),
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, depth=50))
    th = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    th.start()
    time.sleep(a.seconds)
    with lock:
        f.close()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
