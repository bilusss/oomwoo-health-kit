"""ROS-free decision logic for the external topic-activity watcher (``topic_heartbeat_node``).

Nothing here reads a clock or imports ROS: callers pass ``now`` in (node clock, seconds).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

DEFAULT_MAX_EMIT_HZ = 20.0
TF_TOPIC = "/tf"
TF_TYPE = "tf2_msgs/msg/TFMessage"
RELIABILITY = ("best_effort", "reliable")
DURABILITY = ("volatile", "transient_local")
DEFAULT_QOS = {"reliability": "best_effort", "durability": "volatile", "depth": 10}
# Slack on the rate limiter: a source at exactly the limit with a few ms of jitter must not lose every other beat.
LIMIT_SLACK = 0.1


class EmitLimiter:
    """Allows at most ``max_hz`` beats per second on average (ROS time).

    The next slot advances by one interval from the previous slot, not from the arrival time, so a 50 Hz source
    limited to 20 Hz yields about 20 Hz rather than 16.7 Hz. A gap longer than one interval restarts the schedule.
    A clock that jumps backwards also restarts it.
    """

    def __init__(self, max_hz: float = DEFAULT_MAX_EMIT_HZ) -> None:
        if not (math.isfinite(max_hz) and max_hz > 0):
            raise ValueError("max_emit_hz must be positive and finite")
        self.interval = 1.0 / max_hz
        self._next: float | None = None

    def allow(self, now: float) -> bool:
        if not math.isfinite(now):
            return False
        if self._next is None or now < self._next - self.interval - 1e-9 or now - self._next >= self.interval:
            self._next = now + self.interval
            return True
        if now >= self._next - LIMIT_SLACK * self.interval:
            self._next += self.interval
            return True
        return False


FUTURE_TOLERANCE_S = 0.01
DEFAULT_QUEUE_SLACK_S = 0.05
DEFAULT_MAX_PUBLISHERS = 1


def stamp_level(now: float, stamp: float | None, max_age: float | None) -> tuple[str, str]:
    """(level, detail) for one message. No ``max_age``: ``ok``.

    With ``max_age`` set, the message is ``degraded`` when its stamp is missing or zero (None), non-finite, older than
    ``max_age``, or in the future by more than 10 ms (a producer on another clock is not a fresh producer).
    """
    if max_age is None:
        return "ok", ""
    if stamp is None:
        return "degraded", "no header stamp"
    age = now - stamp
    if not math.isfinite(age):
        return "degraded", "non-finite stamp"
    if age > max_age:
        return "degraded", f"stamp age {age:.3f}s > {max_age:g}s"
    if age < -FUTURE_TOLERANCE_S:
        return "degraded", f"stamp {-age:.3f}s in the future"
    return "ok", ""


def queued_too_long(age_s: float | None, max_emit_hz: float, slack_s: float = DEFAULT_QUEUE_SLACK_S) -> bool:
    """True when a message waited in the subscription queue longer than 1/max_emit_hz + slack.

    ``age_s`` is wall-clock now minus the RMW receive (else source) timestamp, or ROS-clock now minus the header stamp
    when neither exists; None means no age source (the message is accepted). A stalled watcher that wakes up must not
    turn old queued messages into fresh heartbeats.
    """
    if age_s is None:
        return False
    if not math.isfinite(age_s):
        return True
    return age_s > 1.0 / max_emit_hz + slack_s


class TtlCache:
    """Caches one value for ``ttl`` seconds; refreshed lazily by the caller (no timer)."""

    def __init__(self, ttl: float = 0.5) -> None:
        self.ttl = ttl
        self._at: float | None = None
        self._value: Any = None

    def get(self, now: float, fetch) -> Any:
        if self._at is None or now < self._at or now - self._at >= self.ttl:
            self._value, self._at = fetch(), now
        return self._value


def publishers_ok(count: int, max_publishers: int) -> tuple[bool, str]:
    if count > max_publishers:
        return False, f"{count} publishers on the topic (max_publishers {max_publishers})"
    return True, ""


class MotionConsistency:
    """Heartbeat condition for drive supervision. Tracks the setpoint the MCU holds (the last value seen on the command topic).

    Beat (healthy) only if
      (a) a NON-ZERO command newer than ``cmd_max_age_s`` exists (the commander is alive; zero commands give no credit), OR
      (b) the held setpoint is zero, or no command was ever seen, AND (odometry below the thresholds continuously for
          ``settle_s`` OR, with a zero setpoint, less than ``coast_max_s`` since the setpoint became zero: the robot may still be
          coasting down).
    A non-zero held setpoint without a fresh command never beats, whatever odometry says (a held creep command below the odometry
    threshold, or odometry that reads zero while the wheels are stalled, must not keep the loop alive). Stale or non-finite
    odometry never beats. A non-finite command value counts as a non-zero setpoint and is not a fresh command.
    """

    def __init__(
        self, linear_threshold: float = 0.02, angular_threshold: float = 0.05,
        cmd_max_age_s: float = 0.15, odom_max_age_s: float = 0.3, settle_s: float = 0.1, coast_max_s: float = 1.0,
    ) -> None:
        for name, v in (("linear_threshold", linear_threshold), ("angular_threshold", angular_threshold),
                        ("cmd_max_age_s", cmd_max_age_s), ("odom_max_age_s", odom_max_age_s),
                        ("settle_s", settle_s), ("coast_max_s", coast_max_s)):
            if not (math.isfinite(v) and v > 0):
                raise ValueError(f"{name} must be positive and finite")
        self.lin_thr, self.ang_thr = linear_threshold, angular_threshold
        self.cmd_max_age_s, self.odom_max_age_s = cmd_max_age_s, odom_max_age_s
        self.settle_s, self.coast_max_s = settle_s, coast_max_s
        self._odom_t: float | None = None
        self._odom_finite = True
        self._quiet_since: float | None = None
        self._cmd_t: float | None = None          # time of the last finite command (None: none, or the last one was non-finite)
        self._held_zero: bool | None = None       # None: no command ever seen
        self._zero_since: float | None = None

    def on_odom(self, now: float, linear: float, angular: float) -> None:
        self._odom_t = now
        self._odom_finite = math.isfinite(now) and math.isfinite(linear) and math.isfinite(angular)
        if self._odom_finite and abs(linear) < self.lin_thr and abs(angular) < self.ang_thr:
            if self._quiet_since is None:
                self._quiet_since = now
        else:
            self._quiet_since = None

    def on_cmd(self, now: float, linear: float = 1.0, angular: float = 0.0, fresh: bool = True) -> None:
        """Called for every message on the command topic with its linear speed and yaw rate (defaults: some non-zero command).

        Only a NON-ZERO command is a sign of a live commander (``fresh``): a zero command sets the held setpoint to zero and only
        starts the settle/coast branch (the coast window starts when the setpoint first becomes zero and is cleared by a non-zero
        command; repeated zeros do not extend it). ``fresh=False`` (the message waited too long in the subscription queue) still
        updates the held setpoint, so it matches what the MCU stand-in forwarded, but gives no freshness credit.
        """
        if not (math.isfinite(now) and math.isfinite(linear) and math.isfinite(angular)):
            self._cmd_t, self._held_zero, self._zero_since = None, False, None
            return
        zero = abs(linear) <= 1e-9 and abs(angular) <= 1e-9
        if zero:
            if self._held_zero is not True:
                self._zero_since = now
            self._held_zero = True
        else:
            self._held_zero, self._zero_since = False, None
            if fresh:
                self._cmd_t = now

    def ok(self, now: float) -> tuple[bool, str]:
        if self._odom_t is None or not math.isfinite(now):
            return False, "no odometry"
        if not self._odom_finite:
            return False, "non-finite odometry"
        if now - self._odom_t > self.odom_max_age_s + 1e-9 or self._odom_t > now:
            return False, f"odometry stale ({now - self._odom_t:.3f}s)"
        if self._cmd_t is not None and 0.0 <= now - self._cmd_t <= self.cmd_max_age_s + 1e-9:
            return True, ""
        if self._held_zero is False:
            return False, f"non-zero setpoint held without a command newer than {self.cmd_max_age_s:g}s"
        if self._quiet_since is not None and now - self._quiet_since >= self.settle_s - 1e-9:
            return True, ""
        if self._held_zero and self._zero_since is not None and 0.0 <= now - self._zero_since < self.coast_max_s:
            return True, ""
        return False, "robot moving or not yet settled with a zero or unknown setpoint"


class TfTracker:
    """True for a transform stamp strictly newer than any seen before."""

    def __init__(self) -> None:
        self._last_ns: int | None = None

    def is_newer(self, stamp_ns: int) -> bool:
        if self._last_ns is not None and stamp_ns <= self._last_ns:
            return False
        self._last_ns = stamp_ns
        return True


def strip_frame(frame: str) -> str:
    return frame.lstrip("/")


@dataclass(frozen=True)
class WatchSpec:
    component_id: str
    topic: str
    msg_type: str
    qos: dict[str, Any]
    max_emit_hz: float
    stamp_max_age_s: float | None
    tf_parent: str | None = None
    tf_child: str | None = None
    max_publishers: int = DEFAULT_MAX_PUBLISHERS
    queue_slack_s: float = DEFAULT_QUEUE_SLACK_S
    motion: dict[str, Any] | None = None  # motion_consistency: odom_topic, cmd_topic, cmd_type, thresholds

    @property
    def is_tf(self) -> bool:
        return self.tf_parent is not None

    @property
    def is_motion(self) -> bool:
        return self.motion is not None


def _qos(raw: Any, where: str) -> dict[str, Any]:
    if raw is None:
        return dict(DEFAULT_QOS)
    if not isinstance(raw, dict):
        raise ValueError(f"{where}: qos must be a mapping")
    unknown = set(raw) - set(DEFAULT_QOS)
    if unknown:
        raise ValueError(f"{where}: unknown qos keys {sorted(unknown)}")
    q = {**DEFAULT_QOS, **raw}
    if q["reliability"] not in RELIABILITY:
        raise ValueError(f"{where}: qos.reliability must be one of {RELIABILITY}")
    if q["durability"] == "transient_local":
        raise ValueError(f"{where}: transient_local is not allowed (a watch must never see replayed messages)")
    if q["durability"] not in DURABILITY:
        raise ValueError(f"{where}: qos.durability must be one of {DURABILITY}")
    if isinstance(q["depth"], bool) or not isinstance(q["depth"], int) or q["depth"] < 1:
        raise ValueError(f"{where}: qos.depth must be an integer >= 1")
    return q


def _positive(raw: Any, where: str, key: str) -> float:
    try:
        v = float(raw)
    except (TypeError, ValueError):
        raise ValueError(f"{where}: {key} must be a number") from None
    if not (math.isfinite(v) and v > 0):
        raise ValueError(f"{where}: {key} must be positive and finite")
    return v


def parse_watches(doc: Any) -> list[WatchSpec]:
    """Validate a loaded YAML document ``{watches: [...]}``."""
    if not isinstance(doc, dict) or not isinstance(doc.get("watches"), list) or not doc["watches"]:
        raise ValueError("config needs a non-empty 'watches' list")
    specs: list[WatchSpec] = []
    seen: set[str] = set()
    for i, w in enumerate(doc["watches"]):
        if not isinstance(w, dict):
            raise ValueError(f"watches[{i}] must be a mapping")
        cid = w.get("component_id")
        if not isinstance(cid, str) or not cid:
            raise ValueError(f"watches[{i}]: component_id is required")
        where = f"watch {cid!r}"
        if cid in seen:
            raise ValueError(f"{where}: duplicate component_id")
        seen.add(cid)
        tf = w.get("tf")
        mc = w.get("motion_consistency")
        motion = None
        if mc is not None:
            if tf is not None or "topic" in w or "type" in w:
                raise ValueError(f"{where}: motion_consistency excludes topic/type/tf")
            if not isinstance(mc, dict) or not mc.get("odom_topic") or not mc.get("cmd_topic"):
                raise ValueError(f"{where}: motion_consistency needs odom_topic and cmd_topic")
            motion = {
                "odom_topic": str(mc["odom_topic"]), "cmd_topic": str(mc["cmd_topic"]),
                "cmd_type": str(mc.get("cmd_type", "geometry_msgs/msg/Twist")),
                "linear_threshold": float(mc.get("linear_threshold", 0.02)),
                "angular_threshold": float(mc.get("angular_threshold", 0.05)),
                "cmd_max_age_s": float(mc.get("cmd_max_age_s", 0.15)),
                "odom_max_age_s": float(mc.get("odom_max_age_s", 0.3)),
                "settle_s": float(mc.get("settle_s", 0.1)),
                "coast_max_s": float(mc.get("coast_max_s", 1.0)),
            }
            MotionConsistency(**{k: v for k, v in motion.items() if k.endswith(("threshold", "_s"))})
            topic, mtype, parent, child = motion["odom_topic"], "nav_msgs/msg/Odometry", None, None
        elif tf is not None:
            if "type" in w:
                raise ValueError(f"{where}: a tf watch has no 'type'")
            if not isinstance(tf, dict) or not tf.get("parent") or not tf.get("child"):
                raise ValueError(f"{where}: tf needs parent and child")
            topic, mtype = str(w.get("topic", TF_TOPIC)), TF_TYPE
            parent, child = strip_frame(str(tf["parent"])), strip_frame(str(tf["child"]))
        else:
            topic, mtype, parent, child = w.get("topic"), w.get("type"), None, None
            if not isinstance(topic, str) or not topic or not isinstance(mtype, str) or not mtype:
                raise ValueError(f"{where}: topic and type (or tf) are required")
        hz = _positive(w.get("max_emit_hz", DEFAULT_MAX_EMIT_HZ), where, "max_emit_hz")
        age = w.get("stamp_max_age_s")
        age = None if age is None else _positive(age, where, "stamp_max_age_s")
        mp = w.get("max_publishers", DEFAULT_MAX_PUBLISHERS)
        if isinstance(mp, bool) or not isinstance(mp, int) or mp < 1:
            raise ValueError(f"{where}: max_publishers must be an integer >= 1")
        slack = _positive(w.get("queue_slack_s", DEFAULT_QUEUE_SLACK_S), where, "queue_slack_s")
        specs.append(WatchSpec(cid, topic, mtype, _qos(w.get("qos"), where), hz, age, parent, child, mp, slack, motion))
    return specs
