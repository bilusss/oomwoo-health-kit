#!/usr/bin/env python3
"""Measure message rates of the watched signals while the robot drives and while it is stationary (M6c, run inside an ohk-sim-<i>).

  python3 probe_rates.py --seconds 120 [--out rates.json]

Wall-clock node (no /clock subscription besides one cheap sample); each message gets the monotonic receive time and the header stamp
(sim). Phase = odom twist above (0.02 m/s, 0.05 rad/s) => "moving", else "stationary". Per topic and phase: count, seconds spent
in the phase, rate (count / phase seconds, wall), rate in sim time (count / sim seconds from header stamps), p50/p95/max gap between
consecutive messages of the same phase (wall, ms). Gaps that straddle a phase change are dropped. At RTF 1.0 wall = sim.
"""

import argparse
import json
import statistics
import threading
import time

import rclpy
from geometry_msgs.msg import PolygonStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

BE = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=50)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    rclpy.init()
    node = Node("ohk_probe_rates")
    lock = threading.Lock()
    st = {"phase": "stationary", "phase_t": time.monotonic(), "time_in": {"moving": 0.0, "stationary": 0.0}}
    ev = {}  # topic -> list of (mono, phase, stamp_sec or None)

    clock = {"t": 0.0}

    def rec(name, stamp=None):
        t = time.monotonic()
        with lock:
            ev.setdefault(name, []).append((t, st["phase"], stamp, None if stamp is None or clock["t"] <= 0 else clock["t"] - stamp))

    def on_odom(m):
        t = time.monotonic()
        v = m.twist.twist
        ph = "moving" if (abs(v.linear.x) >= 0.02 or abs(v.angular.z) >= 0.05) else "stationary"
        with lock:
            st["time_in"][st["phase"]] += t - st["phase_t"]
            st["phase"], st["phase_t"] = ph, t
            s_ = m.header.stamp.sec + m.header.stamp.nanosec / 1e9
            ev.setdefault("/odom", []).append((t, ph, s_, None if clock["t"] <= 0 else clock["t"] - s_))

    last_tf = [0.0]

    def on_tf(m):
        # slam_toolbox republishes map->odom at 50 Hz but advances the stamp only per scan: count distinct (newer) stamps only
        for tr in m.transforms:
            if tr.header.frame_id.lstrip("/") == "map" and tr.child_frame_id.lstrip("/") == "odom":
                s_ = tr.header.stamp.sec + tr.header.stamp.nanosec / 1e9
                if s_ > last_tf[0]:
                    last_tf[0] = s_
                    rec("tf map->odom (new stamp)", s_)
                else:
                    rec("tf map->odom (all msgs)")

    from rosgraph_msgs.msg import Clock
    node.create_subscription(Clock, "/clock", lambda m: clock.__setitem__("t", m.clock.sec + m.clock.nanosec / 1e9), BE)

    node.create_subscription(Odometry, "/odom", on_odom, BE)
    node.create_subscription(LaserScan, "/scan", lambda m: rec("/scan", m.header.stamp.sec + m.header.stamp.nanosec / 1e9), BE)
    node.create_subscription(TFMessage, "/tf", on_tf, QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=100))
    node.create_subscription(PoseWithCovarianceStamped, "/pose",
                             lambda m: rec("/pose", m.header.stamp.sec + m.header.stamp.nanosec / 1e9), BE)
    for t in ("/local_costmap/published_footprint", "/global_costmap/published_footprint"):
        node.create_subscription(PolygonStamped, t, lambda m, t=t: rec(t, m.header.stamp.sec + m.header.stamp.nanosec / 1e9), BE)
    for t in ("/cmd_vel", "/cmd_vel_nav", "/cmd_vel_smoothed", "/cmd_vel_gated"):
        node.create_subscription(Twist, t, lambda m, t=t: rec(t), BE)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    t0 = time.monotonic()
    time.sleep(a.seconds)
    t1 = time.monotonic()
    with lock:
        st["time_in"][st["phase"]] += t1 - st["phase_t"]
        out = {"seconds": round(t1 - t0, 1), "time_in_phase_s": {k: round(v, 1) for k, v in st["time_in"].items()}, "topics": {}}
        for name, rows in ev.items():
            out["topics"][name] = {}
            for ph in ("moving", "stationary"):
                r = [x for x in rows if x[1] == ph]
                tin = st["time_in"][ph]
                gaps = [(b[0] - c[0]) * 1000 for c, b in zip(rows, rows[1:]) if c[1] == ph and b[1] == ph]
                stamps = [x[2] for x in r if x[2] is not None]
                ages = sorted(x[3] for x in r if len(x) > 3 and x[3] is not None)
                d = {"n": len(r), "rate_hz_wall": round(len(r) / tin, 2) if tin > 1 else None}
                if len(stamps) > 2 and stamps[-1] > stamps[0]:
                    # sim-time rate over the covered stamp span only when the phase was continuous: use median stamp period
                    per = sorted(b - c for c, b in zip(stamps, stamps[1:]) if b > c)
                    d["median_stamp_period_sim_s"] = round(statistics.median(per), 4)
                if ages:
                    d.update(sim_now_minus_stamp_s_p50=round(ages[len(ages) // 2], 3), min=round(ages[0], 3), max=round(ages[-1], 3))
                if gaps:
                    gs = sorted(gaps)
                    d.update(gap_ms_p50=round(gs[len(gs) // 2], 1), gap_ms_p95=round(gs[int(len(gs) * 0.95)], 1), gap_ms_max=round(gs[-1], 1))
                out["topics"][name][ph] = d
    print(json.dumps(out, indent=1))
    if a.out:
        json.dump(out, open(a.out, "w"), indent=1)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
