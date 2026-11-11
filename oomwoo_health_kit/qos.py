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
    """Component heartbeats: reliable, volatile (never replayed), depth 10 (all components share one topic)."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def mcu_heartbeat_qos() -> QoSProfile:
    """Aggregate MCU heartbeat: reliable, volatile, depth 1 (a stalled subscriber must not catch up on old beats)."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
