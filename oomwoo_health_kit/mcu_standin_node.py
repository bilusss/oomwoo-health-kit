"""rclpy adapter around McuStandin: heartbeat watchdog + cmd_vel gate."""

import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from oomwoo_health_kit.mcu_standin import McuState, McuStandin, Velocity
from oomwoo_health_kit.qos import latched_state_qos, live_heartbeat_qos

HEARTBEAT_TOPIC = "oomwoo/health/mcu_heartbeat"
STATE_TOPIC = "oomwoo/health/mcu_state"


class McuStandinNode(Node):
    def __init__(self) -> None:
        super().__init__("mcu_standin")
        self.declare_parameter("clock_source", "steady")
        self.declare_parameter("soft_stop_timeout_s", 0.15)
        self.declare_parameter("cpu_reset_timeout_s", 300.0)
        self.declare_parameter("rearm_count", 3)
        src = self.get_parameter("clock_source").value
        if src not in ("steady", "ros"):
            raise ValueError("clock_source must be 'steady' or 'ros'")
        self._ros_clock = src == "ros"
        self._core = McuStandin(
            float(self.get_parameter("soft_stop_timeout_s").value),
            float(self.get_parameter("cpu_reset_timeout_s").value),
            int(self.get_parameter("rearm_count").value),
        )
        self._seen = 0
        self._frozen_reported = False
        self._last_ros_ns = None
        self._last_advance = time.monotonic()
        self._cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self._state_pub = self.create_publisher(String, STATE_TOPIC, latched_state_qos())
        self.create_subscription(String, HEARTBEAT_TOPIC, self._on_heartbeat, live_heartbeat_qos())
        self.create_subscription(Twist, "cmd_vel_in", self._on_cmd, 10)
        self._publish_state("start")
        self.create_timer(0.01, self._on_timer)

    def _now(self) -> float:
        if self._ros_clock:
            return self.get_clock().now().nanoseconds / 1e9
        return time.monotonic()

    def _clock_frozen(self) -> bool:
        """clock_source=ros only: ROS time not advancing for > soft stop of steady time."""
        if not self._ros_clock:
            return False
        ns = self.get_clock().now().nanoseconds
        mono = time.monotonic()
        if ns != self._last_ros_ns:
            self._last_ros_ns = ns
            self._last_advance = mono
        return mono - self._last_advance > self._core.soft_stop_timeout_s

    def _check_clock(self) -> bool:
        frozen = self._clock_frozen()
        if frozen != self._frozen_reported:
            self._frozen_reported = frozen
            if frozen:
                self._cmd_pub.publish(Twist())
            self._publish_state("ros clock frozen" if frozen else "ros clock advancing")
        return frozen

    def _on_heartbeat(self, _msg: String) -> None:
        self._core.heartbeat(self._now())
        self._sync()

    def _on_timer(self) -> None:
        self._check_clock()
        self._core.update(self._now())
        self._sync()

    def _on_cmd(self, msg: Twist) -> None:
        frozen = self._check_clock()
        cmd = Velocity(msg.linear.x, msg.angular.z)
        out = self._core.gate(self._now(), cmd)
        self._sync()
        ok = not frozen and out == cmd and self._core.state is McuState.RUN
        self._cmd_pub.publish(msg if ok else Twist())

    def _sync(self) -> None:
        """Report every transition since the last call; zero Twist per non-RUN entry."""
        new = self._core.transitions[self._seen :]
        self._seen += len(new)
        for t in new:
            if t.to_state is not McuState.RUN:
                self._cmd_pub.publish(Twist())
            self._publish_state(t.reason, t.to_state, t.from_state)

    def _publish_state(self, reason: str, state=None, from_state=None) -> None:
        c = self._core
        state = state or c.state
        payload = {
            "state": state.value,
            "from": from_state.value if from_state else None,
            "reason": reason,
            "reset_asserted": state is McuState.CPU_RESET,
            "clock_fault": self._frozen_reported,
            "transitions": len(c.transitions),
            "soft_stops": c.soft_stops,
            "resets": c.resets,
            "stamp": self._now(),
        }
        self._state_pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = McuStandinNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
