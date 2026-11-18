#!/usr/bin/env python3
"""Raw trace of the in-node heartbeats for M9 recon (run inside an ohk-sim-<i> while a stack and a workload run).

  python3 beat_trace.py --seconds 2000 --out /results/beat_trace_dock.jsonl

One JSON line per message, flushed as it goes (receive time = host monotonic clock, sim time = the /clock value at receipt):
  ["beat", t_mono, sim_now, component_id, stamp_sec, sequence, health, detail]   /oomwoo/health/component (QoS of the contract, depth 200)
  ["odom", t_mono, v, w]                                                         /odom twist, 10 Hz sub-sampled
  ["state", t_mono, sim_now, "BACK"]                                             /dock_drive/state (published on change only)
Analysis: beat_stats.py. Only volatile subscriptions; nothing here changes the graph (the extra subscriber on the shared heartbeat topic is
the same kind of reader as the aggregator).
"""

import argparse
import json
import threading
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

REL = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=200)
BE = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=600.0)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    rclpy.init()
    node = Node("ohk_beat_trace")
    lock = threading.Lock()
    f = open(a.out, "w")
    sim = [0.0]

    def w(row):
        with lock:
            f.write(json.dumps(row, separators=(",", ":")) + "\n")
            f.flush()

    def on_beat(m):
        t, s = time.monotonic(), sim[0]
        try:
            j = json.loads(m.data)
            w(["beat", t, s, j.get("component_id"), j.get("stamp_sec"), j.get("sequence"), j.get("health"), j.get("detail")])
        except ValueError:
            w(["beat_bad", t, s, m.data[:80]])

    last_odom = [0.0]

    def on_odom(m):
        t = time.monotonic()
        if t - last_odom[0] >= 0.1:
            last_odom[0] = t
            w(["odom", t, round(m.twist.twist.linear.x, 4), round(m.twist.twist.angular.z, 4)])

    node.create_subscription(Clock, "/clock", lambda m: sim.__setitem__(0, m.clock.sec + m.clock.nanosec / 1e9), BE)
    node.create_subscription(String, "/oomwoo/health/component", on_beat, REL)
    node.create_subscription(Odometry, "/odom", on_odom, BE)
    node.create_subscription(String, "/dock_drive/state", lambda m: w(["state", time.monotonic(), sim[0], m.data]), REL)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    time.sleep(a.seconds)
    with lock:
        f.close()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
