"""External topic-activity watcher: one component heartbeat per fresh message seen on a watched topic.

Use it for nodes that cannot be modified (Nav2, slam_toolbox, ros_gz_bridge). Heartbeats come from the subscription
callbacks of the watched topics and never from a timer, so when every publisher on the topic is silent and no queued
messages remain, the heartbeats stop. Messages that waited in the queue too long (a stalled watcher) and messages
on a topic with more publishers than ``max_publishers`` give no heartbeat. It proves that a node produces output,
not that the output is correct.

Queue-age guard (a stalled watcher must not turn old queued messages into fresh heartbeats): age = host wall clock now minus
the RMW SOURCE timestamp (set by the publisher, measured on the real stack: ros_gz_bridge, Nav2 and slam_toolbox all set it).
The receive timestamp is NOT used (Fast DDS takes it at take time, it cannot show queueing). If the source timestamp is zero the
header stamp is used (message age against this node's clock, so the node clock must be the stamp clock); if there is neither,
the guard cannot work for that message: it is logged once per watch and every such heartbeat carries the detail
"queue guard unavailable". Assumption: publisher and watcher share one host clock (source timestamps are wall time of the publisher's host).
Watchers of ``tf`` pairs have no writer guard: /tf has many writers by design, a second writer of the same (parent, child) pair
(or a surviving duplicate of a restarted node) is not detected there.
"""

import math
import time

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

from oomwoo_health_kit.ros_heartbeat import RosHeartbeat
from oomwoo_health_kit.topic_watch import (
    EmitLimiter, MotionConsistency, TfTracker, TtlCache, WatchSpec,
    parse_watches, publishers_ok, queued_too_long, stamp_level, strip_frame,
)


def _qos_profile(q: dict) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=q["depth"],
        reliability=ReliabilityPolicy.BEST_EFFORT if q["reliability"] == "best_effort" else ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE if q["durability"] == "volatile" else DurabilityPolicy.TRANSIENT_LOCAL,
    )


def _stamp_sec(stamp) -> float | None:
    """Seconds, or None for an unset (all-zero) stamp."""
    if stamp.sec == 0 and stamp.nanosec == 0:
        return None
    return stamp.sec + stamp.nanosec / 1e9


class TopicHeartbeat(Node):
    def __init__(self) -> None:
        super().__init__("topic_heartbeat")
        self.declare_parameter("config_file", "")
        path = self.get_parameter("config_file").value
        if not path:
            raise SystemExit("topic_heartbeat: parameter config_file is required")
        with open(path) as f:
            specs = parse_watches(yaml.safe_load(f))
        self._subs = []
        self._warned: dict[str, float] = {}
        for s in specs:
            self._watch(s)
        self.get_logger().info("watching " + ", ".join(f"{s.component_id}<-{s.topic}" for s in specs))

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _warn(self, key: str, text: str) -> None:
        now = time.monotonic()
        if now - self._warned.get(key, -1e9) > 5.0:
            self._warned[key] = now
            self.get_logger().warn(text)

    @staticmethod
    def _queue_age(info, header_stamp_sec, now_ros) -> tuple[float | None, str]:
        """(seconds since the publisher sent the message, basis). Basis 'source' (RMW source timestamp), 'header' (zero source
        stamp: node clock minus header stamp) or 'none' (no way to know)."""
        ts = (info or {}).get("source_timestamp", 0)
        if ts and ts > 0:
            return time.time() - ts / 1e9, "source"
        if header_stamp_sec is not None:
            return now_ros - header_stamp_sec, "header"
        return None, "none"

    def _fresh(self, spec: WatchSpec, info, header_stamp_sec, now) -> tuple[bool, str]:
        """(fresh enough, detail). detail is non-empty when the queue guard could not work for this message."""
        age, basis = self._queue_age(info, header_stamp_sec, now)
        detail = ""
        if basis == "none":
            detail = "queue guard unavailable (zero source timestamp, no header stamp)"
            self._warn(spec.component_id + ":guard", f"{spec.component_id}: {detail}: a stalled watcher could replay queued messages")
        elif basis == "header" and self._warned.get(spec.component_id + ":hdr") is None:
            self._warned[spec.component_id + ":hdr"] = time.monotonic()
            self.get_logger().warn(f"{spec.component_id}: zero source timestamp, queue age taken from the header stamp (node clock)")
        return not queued_too_long(age, spec.max_emit_hz, spec.queue_slack_s), detail

    def _single_writer(self, spec: WatchSpec, cache: TtlCache, topic: str, now: float) -> bool:
        count = cache.get(now, lambda: len(self.get_publishers_info_by_topic(self.resolve_topic_name(topic))))
        ok, detail = publishers_ok(count, spec.max_publishers)
        if not ok:
            self._warn(spec.component_id, f"{spec.component_id}: no heartbeat, {detail}")
        return ok

    def _watch(self, spec: WatchSpec) -> None:
        hb = RosHeartbeat(self, spec.component_id)
        limiter = EmitLimiter(spec.max_emit_hz)
        qos = _qos_profile(spec.qos)
        pubs = TtlCache(0.5)

        def stamp_of(stamp):
            return _stamp_sec(stamp) if stamp is not None else None

        def emit(stamp, extra: str = "") -> None:
            now = self._now()
            if limiter.allow(now):
                level, detail = stamp_level(now, stamp_of(stamp), spec.stamp_max_age_s)
                hb.beat(level, "; ".join(x for x in (detail, extra) if x))

        if spec.is_motion:
            self._watch_motion(spec, hb, limiter, qos, pubs)
            return

        if spec.is_tf:
            tracker = TfTracker()
            msg_cls = get_message(spec.msg_type)

            def on_tf(msg, info) -> None:
                # Matches one (parent, child) pair, so other tf publishers cannot stand in for it.
                now = self._now()
                for t in msg.transforms:
                    if strip_frame(t.header.frame_id) == spec.tf_parent and strip_frame(t.child_frame_id) == spec.tf_child:
                        fresh, note = self._fresh(spec, info, stamp_of(t.header.stamp), now)
                        if not fresh:
                            continue
                        ns = t.header.stamp.sec * 1_000_000_000 + t.header.stamp.nanosec
                        if tracker.is_newer(ns):
                            emit(t.header.stamp, note)

            self._subs.append(self.create_subscription(msg_cls, spec.topic, on_tf, qos))
            return

        msg_cls = get_message(spec.msg_type)

        def on_msg(msg, info) -> None:
            now = self._now()
            header = getattr(msg, "header", None)
            stamp = header.stamp if header is not None else None
            fresh, note = self._fresh(spec, info, stamp_of(stamp), now)
            if not fresh:
                return
            if not self._single_writer(spec, pubs, spec.topic, now):
                return
            emit(stamp, note)

        self._subs.append(self.create_subscription(msg_cls, spec.topic, on_msg, qos))

    def _watch_motion(self, spec, hb, limiter, qos, pubs) -> None:
        m = spec.motion
        core = MotionConsistency(m["linear_threshold"], m["angular_threshold"], m["cmd_max_age_s"], m["odom_max_age_s"],
                                 m["settle_s"], m["coast_max_s"])
        odom_cls, cmd_cls = get_message("nav_msgs/msg/Odometry"), get_message(m["cmd_type"])
        # The command topic carries the setpoint the MCU holds: reliable (a lost message would leave a stale setpoint), and it
        # must be the topic the MCU stand-in receives (the last hop; any writer counts, max_publishers is not applied).
        cmd_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
        notes = {"odom": "", "cmd": ""}

        def maybe_beat(now: float) -> None:
            ok, detail = core.ok(now)
            if ok and limiter.allow(now):
                hb.beat("ok", "; ".join(x for x in notes.values() if x))
            elif not ok:
                self._warn(spec.component_id, f"{spec.component_id}: no heartbeat, {detail}")

        def on_odom(msg, info) -> None:
            now = self._now()
            fresh, notes["odom"] = self._fresh(spec, info, _stamp_sec(msg.header.stamp), now)
            if not fresh or not self._single_writer(spec, pubs, m["odom_topic"], now):
                return
            t = msg.twist.twist
            core.on_odom(now, math.hypot(t.linear.x, t.linear.y), t.angular.z)
            maybe_beat(now)

        def on_cmd(msg, info) -> None:
            now = self._now()
            fresh, notes["cmd"] = self._fresh(spec, info, None, now)
            lin, ang = msg.linear, msg.angular
            # a command that waited too long still updates the held setpoint (the stand-in forwarded it), without freshness credit
            core.on_cmd(now, math.hypot(lin.x, lin.y, lin.z), math.hypot(ang.x, ang.y, ang.z), fresh=fresh)
            if fresh:
                maybe_beat(now)

        self._subs.append(self.create_subscription(odom_cls, m["odom_topic"], on_odom, qos))
        self._subs.append(self.create_subscription(cmd_cls, m["cmd_topic"], on_cmd, cmd_qos))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TopicHeartbeat()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
