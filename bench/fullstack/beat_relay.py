#!/usr/bin/env python3
"""Sim-clock bridge for the in-node heartbeats (M9). Runs only in the sim bench, never on a robot.

Why it exists: the in-node heartbeats are stamped with the publishing node's ROS clock, which in Gazebo is /clock (sim time). The aggregator
treats a heartbeat that is NEWER than its own clock as stale (age < 0). First M9 smoke run with everything on sim time: 28 soft stops in 240 sim s,
27 blamed on dock_drive alone, with a 5 s max_age. Cause (found in the M9 review, docs/M9_RECON.md section 3): not a late /clock tick but float
rounding on an EQUAL tick: the nodes stamped nanoseconds * 1e-9 (oomwoo-ros2-tools 40552ff), the aggregator reads nanoseconds / 1e9, and the
product is 1 ulp larger for about 41% of values; dock_drive and the aggregator run 20 Hz timers on the same /clock ticks. Fixed in the patch by
cb088fd. The relay was used for all M9 data (its clamp below never fired: clamped=0 in every logged counter). On the robot the aggregator reads its
clock after the beat was stamped, so this cannot happen there.

What it does: the instrumented nodes are launched with /oomwoo/health/component remapped to /oomwoo/health/component_node. This node (on sim time)
subscribes there, and for each beat republishes the same JSON on /oomwoo/health/component with stamp_sec = wall_now - max(0, sim_now - stamp_sec)
(the beat's age in sim seconds, never negative, moved onto the wall clock the aggregator and watcher use, as in M7/M8). Only beats that a node
actually published are forwarded, with the node's own sequence, health and detail; a dead or wedged node still goes stale, and a dead relay makes
every in-node component stale (fail-safe). At RTF 1.0 sim and wall seconds agree. QoS on both sides is the contract's (reliable, volatile, 10).
"""

import json
import time

NODE_TOPIC = "/oomwoo/health/component_node"
OUT_TOPIC = "/oomwoo/health/component"


def convert(payload, sim_now, wall_now):
    """Payload dict as published by the node + the relay's sim and wall clock at receipt -> payload for the aggregator.

    stamp_sec becomes wall_now - age with age = max(0, sim_now - stamp_sec); everything else is kept. Raises KeyError / TypeError / ValueError
    for a payload without a numeric stamp_sec (the aggregator would drop it as well)."""
    stamp = float(payload["stamp_sec"])
    if stamp != stamp or stamp in (float("inf"), float("-inf")):
        raise ValueError("stamp_sec is not finite")
    out = dict(payload)
    out["stamp_sec"] = float(wall_now - max(0.0, sim_now - stamp))
    return out


class BeatRelay:
    """The relay's whole logic, ROS-free: one input message gives at most one output message, immediately; there is no timer, no stored
    beat and no way to emit without an input, so a node that stops beating stops being forwarded (tested in test_fullstack_beat_stats.py)."""

    def __init__(self):
        self.n = self.bad = self.clamped = 0

    def on_beat(self, raw, sim_now, wall_now):
        """Raw JSON text from the node -> JSON text for the aggregator, or None for a malformed beat (dropped, counted)."""
        try:
            p = json.loads(raw)
            out = convert(p, sim_now, wall_now)
        except (ValueError, KeyError, TypeError):
            self.bad += 1
            return None
        self.n += 1
        if sim_now < float(p["stamp_sec"]):
            self.clamped += 1   # the relay's own /clock was behind the sender's: age 0 instead of negative
        return json.dumps(out, sort_keys=True)


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    rclpy.init()
    node = Node("beat_relay", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
    qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
    pub = node.create_publisher(String, OUT_TOPIC, qos)
    relay = BeatRelay()

    def on_beat(m):
        out = relay.on_beat(m.data, node.get_clock().now().nanoseconds * 1e-9, time.time())
        if out is None:
            node.get_logger().warn("dropping malformed beat", throttle_duration_sec=5.0)
        else:
            pub.publish(String(data=out))

    node.create_subscription(String, NODE_TOPIC, on_beat, qos)
    node.create_timer(60.0, lambda: node.get_logger().info(f"beat_relay: n={relay.n} bad={relay.bad} clamped={relay.clamped}"))
    node.get_logger().info(f"beat_relay: {NODE_TOPIC} -> {OUT_TOPIC} (sim age moved onto the wall clock)")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
