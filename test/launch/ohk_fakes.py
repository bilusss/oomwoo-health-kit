"""Stand-ins for real nodes (lidar bridge, odometry, tf, controller output, costmap) for the nav watcher tests.

One process per publisher so each can be SIGSTOPped or SIGKILLed on its own.
    ohk_fakes.py --kind {scan,odom,twist,tf,polygon,pose,smoother} --topic T --hz H [--enable-topic E] [--vx V] [--lag-s L] [--stale-s S]
M8: kind pose publishes PoseStamped (a sensor fix, e.g. the dock detector). kind smoother imitates Nav2's velocity smoother: it
subscribes --in-topic (Twist), holds the last setpoint and re-emits it at --hz on --topic and --also-topic; after --timeout-s without an
input message, or when the input is zero, it ramps the held value to zero in --decel-s and then publishes zeros while an output
was non-zero (like the real stage: silent at rest).
SIGUSR1 toggles a stale header stamp (now - S) on scan/odom/polygon messages; SIGUSR2 toggles odom motion off/on.
--enable-topic (std_msgs/String "on"/"off", transient-local): a twist publishes only while "on" (linear.x = --value) and sends ONE
zero Twist when switched off after having been on (like Nav2 ending a command stream with a zero); odom reports linear
velocity V while "on" and for --lag-s after "off" (the robot coasting), 0 otherwise. Without it odom reports V always.
"""

import argparse
import signal

import rclpy
from geometry_msgs.msg import PolygonStamped, PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

import time


class Smoother(Node):
    """Velocity smoother imitation (see the module docstring)."""

    def __init__(self, a) -> None:
        super().__init__("fake_smoother")
        reliable = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.a = a
        self.pubs = [self.create_publisher(Twist, t, reliable) for t in [a.topic, *([a.also_topic] if a.also_topic else [])]]
        self.create_subscription(Twist, a.in_topic, self._on_in, reliable)
        self.target = 0.0        # last input value (linear x)
        self.cur = 0.0           # emitted value
        self.last_in = None
        self.emitting = False
        self.create_timer(1.0 / a.hz, self._tick)

    def _on_in(self, m) -> None:
        self.target = m.linear.x
        self.last_in = time.monotonic()

    def _tick(self) -> None:
        now = time.monotonic()
        tgt = self.target if (self.last_in is not None and now - self.last_in <= self.a.timeout_s) else 0.0
        step = abs(self.a.value) / max(self.a.decel_s, 1e-3) / self.a.hz   # full-scale ramp in decel_s
        if tgt > self.cur:
            self.cur = min(tgt, self.cur + step)
        else:
            self.cur = max(tgt, self.cur - step)
        if abs(self.cur) > 1e-9:
            self.emitting = True
        elif not self.emitting:
            return                      # silent at rest
        m = Twist()
        m.linear.x = self.cur
        for p in self.pubs:
            p.publish(m)
        if abs(self.cur) <= 1e-9:
            self.emitting = False       # one zero after the ramp, then silent


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
            "pose": (PoseStamped, reliable),
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
            if self.a.kind == "twist":
                self.pub.publish(Twist())
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
            m.linear.x = self.a.value
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
        elif k == "pose":
            m = PoseStamped()
            m.header.stamp, m.header.frame_id = self._stamp(), "laser"
            m.pose.orientation.w = 1.0
        else:
            m = PolygonStamped()
            m.header.stamp, m.header.frame_id = self._stamp(), "base"
        self.pub.publish(m)


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--kind", required=True, choices=["scan", "odom", "twist", "tf", "polygon", "pose", "smoother"])
    p.add_argument("--topic", required=True)
    p.add_argument("--hz", type=float, required=True)
    p.add_argument("--enable-topic", default=None)
    p.add_argument("--vx", type=float, default=0.0)
    p.add_argument("--value", type=float, default=0.2, help="twist: linear.x while enabled")
    p.add_argument("--lag-s", type=float, default=0.1)
    p.add_argument("--stale-s", type=float, default=2.0)
    p.add_argument("--in-topic", default="/nav_cmd", help="smoother: input topic")
    p.add_argument("--also-topic", default=None, help="smoother: second output topic (the stand-in input)")
    p.add_argument("--timeout-s", type=float, default=1.0, help="smoother: velocity_timeout")
    p.add_argument("--decel-s", type=float, default=0.3, help="smoother: seconds to ramp full scale (--value) to zero")
    a, _ = p.parse_known_args()
    rclpy.init()
    node = Smoother(a) if a.kind == "smoother" else Fake(a)
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
