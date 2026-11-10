"""Publishes one latched roster."""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from oomwoo_health_kit.qos import latched_state_qos


class RosterPublisher(Node):
    def __init__(self) -> None:
        super().__init__("roster_publisher")
        self.declare_parameter("task_id", "ohk_task")
        self.declare_parameter("critical_ids", ["critical_a"])
        self.declare_parameter("advisory_ids", ["advisory_a"])
        self.declare_parameter("max_age_sec", 0.15)
        age = float(self.get_parameter("max_age_sec").value)
        comps = [
            {"component_id": c, "critical": crit, "max_age_sec": age}
            for crit, key in ((True, "critical_ids"), (False, "advisory_ids"))
            for c in self.get_parameter(key).value
        ]
        self._pub = self.create_publisher(String, "oomwoo/health/roster", latched_state_qos())
        payload = {"task_id": self.get_parameter("task_id").value, "components": comps}
        self._pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RosterPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
