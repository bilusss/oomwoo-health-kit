"""Stand-ins for real nodes (lidar bridge, odometry, tf, controller output, costmap) for the nav watcher tests.

One process per publisher so each can be SIGSTOPped or SIGKILLed on its own.
    ohk_fakes.py --kind {scan,odom,twist,tf,polygon} --topic T --hz H [--enable-topic E] [--vx V] [--lag-s L] [--stale-s S]
SIGUSR1 toggles a stale header stamp (now - S) on scan/odom/polygon messages; SIGUSR2 toggles odom motion off/on.
--enable-topic (std_msgs/String "on"/"off", transient-local): a twist publishes only while "on"; odom reports linear
velocity V while "on" and for --lag-s after "off" (the robot coasting), 0 otherwise. Without it odom reports V always.
"""

import argparse
import signal

import rclpy
from geometry_msgs.msg import PolygonStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

import time


class Fake(Node):
    def __init__(self, a) -> None:
        super().__init__(f"fake_{a.kind}_{a.topic.strip('/').replace('/', '_')}")
        self.a = a
        self.stale = False
        self.enabled = a.enable_topic is None
        self.off_at = None
        self.halted = False
        signal.signal(signal.SIGUSR1, lambda *_: setattr(self, "stale", not self.stale))
        signal.signal(signal.SIGUSR2, lambda *_: setattr(self, "halted", not self.halted))
        best_effort = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE)
        cls, qos = {
            "scan": (LaserScan, best_effort),  # sensor topic, best-effort like the real bridge
            "odom": (Odometry, reliable),
            "twist": (Twist, reliable),
            "tf": (TFMessage, reliable),
            "polygon": (PolygonStamped, reliable),
        }[a.kind]
        self.pub = self.create_publisher(cls, a.topic, qos)
        if a.enable_topic:
            tl = QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=1,
                reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(String, a.enable_topic, self._on_enable, tl)
        self.create_timer(1.0 / a.hz, self._tick)

    def _on_enable(self, m) -> None:
        on = m.data == "on"
        if self.enabled and not on:
            self.off_at = time.monotonic()
        self.enabled = on

    def _moving(self) -> bool:
        if self.halted:
            return False
        if self.a.enable_topic is None or self.enabled:
            return True
        return self.off_at is not None and time.monotonic() - self.off_at < self.a.lag_s

    def _stamp(self):
        t = self.get_clock().now()
        if self.stale:
            t = t - Duration(seconds=self.a.stale_s)
        return t.to_msg()

    def _tick(self) -> None:
        k = self.a.kind
        if k == "twist" and not self.enabled:
            return
        if k == "twist":
            m = Twist()
            m.linear.x = 0.2
        elif k == "tf":
            tr = TransformStamped()
            tr.header.stamp = self.get_clock().now().to_msg()
            tr.header.frame_id, tr.child_frame_id = "map", "odom"
            tr.transform.rotation.w = 1.0
            m = TFMessage(transforms=[tr])
        elif k == "scan":
            m = LaserScan()
            m.header.stamp, m.header.frame_id = self._stamp(), "laser"
            m.ranges = [1.0] * 8
        elif k == "odom":
            m = Odometry()
            m.header.stamp, m.header.frame_id = self._stamp(), "odom"
            m.twist.twist.linear.x = self.a.vx if self._moving() else 0.0
        else:
            m = PolygonStamped()
            m.header.stamp, m.header.frame_id = self._stamp(), "base"
        self.pub.publish(m)


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--kind", required=True, choices=["scan", "odom", "twist", "tf", "polygon"])
    p.add_argument("--topic", required=True)
    p.add_argument("--hz", type=float, required=True)
    p.add_argument("--enable-topic", default=None)
    p.add_argument("--vx", type=float, default=0.0)
    p.add_argument("--lag-s", type=float, default=0.1)
    p.add_argument("--stale-s", type=float, default=2.0)
    a, _ = p.parse_known_args()
    rclpy.init()
    node = Fake(a)
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
