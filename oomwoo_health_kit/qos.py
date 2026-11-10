"""QoS profiles from the stack watchdog contract."""

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def latched_state_qos() -> QoSProfile:
    """Roster, stack and stand-in state: reliable, transient-local, depth 1."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def live_heartbeat_qos() -> QoSProfile:
    """Component and MCU heartbeats: reliable, volatile (never replayed)."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
