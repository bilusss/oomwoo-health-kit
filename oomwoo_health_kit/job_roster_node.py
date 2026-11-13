"""Optional job-following roster publisher (the shipped default is a static roster; see docs/SPEC.md, M6a).

Publishes the roster for the current job state, latched (transient-local), re-posted on transitions. State sources:
``static`` (one set, posted once), ``topic`` (std_msgs/String naming a set) and ``nav2_actions`` (any active goal on the
configured action status topics selects ``active_state``). It starts in, and falls back to, the strictest set; stricter
sets are applied at once, relaxation is debounced.

Plumbing liveness: with a dynamic source the node also publishes component ``job_roster`` (critical in every set).
Its work cycle is the re-validation done on every poll tick (20 ms): the state source is still readable (every status
topic has a publisher), the posted roster is the one for the committed state. A beat (10 Hz) is level ``ok`` only when
that validation passes, otherwise ``degraded``. A dead or wedged node stops beating, so the aggregator stops the robot
after max_age + stand-in soft stop. This proves the node's loop runs and that the status topics still have a MATCHED publisher
endpoint, not that the job state is right and not that the action server is alive: a SIGKILLed action server stays matched
until its DDS lease expires (about 20 s with Fast DDS defaults) and a SIGSTOPped one stays matched indefinitely. What limits the
damage of a dead or frozen Nav2 is drive_supervision (motion without a fresh command), not this check.
"""

import json

import rclpy
import yaml
from action_msgs.msg import GoalStatusArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from oomwoo_health_kit.job_logic import PLUMBING_ID, Debouncer, StatusSource, any_active, parse_job_config
from oomwoo_health_kit.qos import latched_state_qos
from oomwoo_health_kit.ros_heartbeat import RosHeartbeat
from oomwoo_health_kit.topic_watch import EmitLimiter

POLL_S = 0.02
GRAPH_REFRESH_S = 0.25
BEAT_HZ = 10.0


class JobRoster(Node):
    def __init__(self) -> None:
        super().__init__("job_roster")
        self.declare_parameter("config_file", "")
        path = self.get_parameter("config_file").value
        if not path:
            raise SystemExit("job_roster: parameter config_file is required")
        with open(path) as f:
            self._cfg = parse_job_config(yaml.safe_load(f))
        c = self._cfg
        self._pub = self.create_publisher(String, "oomwoo/health/roster", latched_state_qos())
        self._deb = Debouncer(c.strictest, c.debounce_s, c.ranks)
        self._posted = c.strictest
        self._post(c.strictest)  # always strictest first, also after a restart in the middle of a goal
        if c.source_type == "static":
            return
        self._hb = RosHeartbeat(self, PLUMBING_ID)
        self._limiter = EmitLimiter(BEAT_HZ)
        self._source = StatusSource(c.status_topics, c.settle_s)
        self._last_graph = -1e9
        self._desired = c.strictest
        tl = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        if c.source_type == "topic":
            self.create_subscription(String, c.job_state_topic, self._on_job_state, tl)
        else:  # rcl_action status topics are reliable, transient-local, depth 1
            self._status_subs = {
                t: self.create_subscription(GoalStatusArray, t, lambda m, t=t: self._on_status(t, m), tl)
                for t in c.status_topics
            }
        self.create_timer(POLL_S, self._poll)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _post(self, state: str) -> None:
        rs = self._cfg.rosters[state]
        self._pub.publish(String(data=json.dumps(rs.payload(), sort_keys=True)))
        self._posted = state
        self.get_logger().info(f"roster {state!r} (task_id {rs.task_id}) posted")

    def _on_job_state(self, msg: String) -> None:
        name = msg.data.strip()
        if name not in self._cfg.rosters:
            self.get_logger().warn(f"unknown job state {name!r}: using the strictest set")
            name = self._cfg.strictest
        self._desired = name
        self._apply(name)

    def _on_status(self, topic: str, msg: GoalStatusArray) -> None:
        self._source.on_status(topic, any_active(s.status for s in msg.status_list))

    def _apply(self, desired: str) -> None:
        new = self._deb.observe(self._now(), desired)
        if new is not None:
            self._post(new)

    def _refresh_graph(self, now: float) -> None:
        if now - self._last_graph >= GRAPH_REFRESH_S or now < self._last_graph:
            self._last_graph = now
            for t, sub in self._status_subs.items():
                # matched publishers (QoS-compatible and discovered), so the latched status is on its way
                self._source.on_publishers(now, t, sub.get_publisher_count())

    def _poll(self) -> None:
        now = self._now()
        c = self._cfg
        valid, detail = True, ""
        if c.source_type == "nav2_actions":
            self._refresh_graph(now)
            valid = self._source.valid()
            detail = "" if valid else "an action status topic has no publisher"
            st = self._source.state(now)
            self._desired = {"active": c.active_state, "inactive": c.inactive_state}.get(st, c.strictest)
            self._apply(self._desired)
        new = self._deb.poll(now)
        if new is not None:
            self._post(new)
        if self._posted != self._deb.committed:  # self-check of the latched roster
            valid, detail = False, "posted roster differs from committed state"
        if self._limiter.allow(now):
            self._hb.beat("ok" if valid else "degraded", detail)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JobRoster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
