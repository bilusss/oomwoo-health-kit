"""Node helper: publishes WorkHeartbeat payloads on the contract topic."""

from std_msgs.msg import String

from oomwoo_health_kit.heartbeat import WorkHeartbeat, to_json
from oomwoo_health_kit.qos import live_heartbeat_qos

COMPONENT_TOPIC = "oomwoo/health/component"


class RosHeartbeat:
    def __init__(self, node, component_id: str, topic: str = COMPONENT_TOPIC) -> None:
        self._node = node
        self._core = WorkHeartbeat(component_id)
        self._pub = node.create_publisher(String, topic, live_heartbeat_qos())

    def beat(self, level: str = "ok", detail: str = "") -> None:
        """Call at the point where a unit of work completed."""
        now = self._node.get_clock().now().nanoseconds / 1_000_000_000.0
        self._pub.publish(String(data=to_json(self._core.beat(now, level, detail))))
