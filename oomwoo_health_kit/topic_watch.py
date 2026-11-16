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


def _pos_finite(*vals: float) -> bool:
    return all(math.isfinite(v) for v in vals)


class CommandChainGuard:
    """M8 rule A: a command stage that re-emits its held setpoint must keep getting commands from upstream.

    Nav2 puts a velocity smoother between its commanders (controller_server, behavior_server) and the stand-in's input. The smoother
    re-emits the last setpoint at its own rate for ``velocity_timeout`` after the commander went silent, so the stream on the
    stand-in input stays fresh and non-zero while the commander is dead (drive_supervision cannot see it). This guard watches both
    sides of the stage: ``on_in`` for the stage's input topic (any writer), ``on_out`` for its output topic.

    Healthy (``ok``) if
      (a) the output is zero, or no output was ever seen (nothing is being replayed), OR
      (b) the output is non-zero AND a non-zero input arrived no more than ``in_max_age_s`` ago (a commander is alive), OR
      (c) the output is non-zero AND the newest input messages are zero (a commanded stop: the stage is decelerating) AND the run of
          zero inputs began less than ``stop_grace_s`` ago. Repeated zero inputs do not extend the grace.
    Non-zero output with no input ever seen, a non-finite value on either side, or a clock that runs backwards is not healthy.
    The guard says nothing about a silent output (a dead stage): the stand-in input watch of drive_supervision covers that.

    Ways the rule could stop working without any fault of the robot (added after the M8 measurements; they only act where the older
    paths said "healthy because nothing was ever seen"; see docs/FINDINGS.md M8):
      * ``set_graph(ok, detail)``: the adapter reports whether the watched topics have publishers of the expected message type; a missing
        publisher or a type mismatch (Twist vs TwistStamped, a remap) is NOT healthy.
      * ``on_odom(now, linear, angular)``: odometry above ``motion_linear`` / ``motion_angular`` for more than ``motion_grace_s`` while NO output
        has ever been seen on the stage is NOT healthy (the stage output is not reaching the watcher, or the robot is moved by a writer that
        bypasses the stage; the rule is then not usable for that stack). Without ``on_odom`` calls this check does nothing.
    """

    def __init__(self, in_max_age_s: float = 0.35, stop_grace_s: float = 1.0, motion_linear: float = 0.02, motion_angular: float = 0.05,
                 motion_grace_s: float = 0.5) -> None:
        for name, v in (("in_max_age_s", in_max_age_s), ("stop_grace_s", stop_grace_s), ("motion_linear", motion_linear),
                        ("motion_angular", motion_angular), ("motion_grace_s", motion_grace_s)):
            if not (math.isfinite(v) and v > 0):
                raise ValueError(f"{name} must be positive and finite")
        self.in_max_age_s, self.stop_grace_s = in_max_age_s, stop_grace_s
        self.motion_linear, self.motion_angular, self.motion_grace_s = motion_linear, motion_angular, motion_grace_s
        self._graph_ok, self._graph_detail = True, ""
        self._moving_since: float | None = None    # start of the current stretch of odometry motion (None: still or no odometry)
        self._in_nz_t: float | None = None      # time of the last non-zero input
        self._in_zero_since: float | None = None  # start of the current run of zero inputs (None: the last input was non-zero or none seen)
        self._in_bad = False                    # the last input was non-finite
        self._out_zero: bool | None = None      # None: no output seen
        self._last_t: float | None = None

    def _seen(self, now: float) -> bool:
        """False (and the guard poisoned for this instant) for a non-finite time."""
        if not math.isfinite(now):
            return False
        self._last_t = now if self._last_t is None else max(self._last_t, now)
        return True

    def set_graph(self, ok: bool, detail: str = "") -> None:
        self._graph_ok, self._graph_detail = bool(ok), detail

    def on_odom(self, now: float, linear: float, angular: float = 0.0) -> None:
        if not (_pos_finite(now, linear, angular) and self._seen(now)):
            self._moving_since = self._moving_since if self._moving_since is not None else (now if math.isfinite(now) else 0.0)
            return      # non-finite odometry counts as motion
        if abs(linear) >= self.motion_linear or abs(angular) >= self.motion_angular:
            if self._moving_since is None:
                self._moving_since = now
        else:
            self._moving_since = None

    def on_in(self, now: float, linear: float, angular: float = 0.0) -> None:
        if not (_pos_finite(now, linear, angular) and self._seen(now)):
            self._in_bad, self._in_nz_t, self._in_zero_since = True, None, None
            return
        self._in_bad = False
        if abs(linear) <= 1e-9 and abs(angular) <= 1e-9:
            if self._in_zero_since is None:
                self._in_zero_since = now
        else:
            self._in_nz_t, self._in_zero_since = now, None

    def on_out(self, now: float, linear: float, angular: float = 0.0) -> None:
        if not (_pos_finite(now, linear, angular) and self._seen(now)):
            self._out_zero = False      # a non-finite output is a non-zero setpoint
            return
        self._out_zero = abs(linear) <= 1e-9 and abs(angular) <= 1e-9

    def ok(self, now: float) -> tuple[bool, str]:
        if not math.isfinite(now):
            return False, "non-finite time"
        if self._last_t is not None and now < self._last_t - 1e-9:
            return False, "clock ran backwards"
        if not self._graph_ok:
            return False, f"stage topics not as expected: {self._graph_detail}"
        if self._out_zero is None:
            if self._moving_since is not None and now - self._moving_since > self.motion_grace_s + 1e-9:
                return False, (f"odometry shows motion for {now - self._moving_since:.2f}s but no output was ever seen on the stage "
                               "(wrong topic or message type, or a writer that bypasses the stage)")
            return True, ""
        if self._out_zero:
            return True, ""
        if self._in_bad:
            return False, "non-finite command on the stage input"
        if self._in_nz_t is not None and 0.0 <= now - self._in_nz_t <= self.in_max_age_s + 1e-9:
            return True, ""
        if self._in_zero_since is not None and 0.0 <= now - self._in_zero_since < self.stop_grace_s:
            return True, ""
        if self._in_nz_t is None and self._in_zero_since is None:
            return False, "stage output non-zero, no input ever seen"
        if self._in_zero_since is not None:
            return False, f"stage output non-zero {now - self._in_zero_since:.2f}s after the input went to zero (grace {self.stop_grace_s:g}s)"
        return False, f"stage output non-zero, no non-zero input newer than {self.in_max_age_s:g}s ({now - self._in_nz_t:.2f}s)"


class ReverseFixGuard:
    """M8 rule B: while the robot is commanded to reverse, the sensor fix the reversing controller depends on must be fresh.

    dock_drive reverses into the bay steering on the detector's dock pose and, when the fixes stop, keeps its estimate by odometry
    for ``pose_timeout_s`` (3 s) before it holds. A dead detector is therefore invisible to the command stream. The guard tracks the
    held setpoint on the command topic (``on_cmd``) and the fix messages (``on_fix``).

    Reversing = the held setpoint has linear x <= -``reverse_threshold`` (a non-finite value counts as reversing). While reversing the
    guard is healthy only if the age of the reference fix is <= ``fix_max_age_s``. With ``from_reverse_start`` the reference is the
    later of the last fix and the start of the current reversing episode (an episode that begins after a long silent stretch of the
    detector then gets ``fix_max_age_s`` to produce its first fix); without it the last fix must be fresh from the first reversing
    command on. Not reversing (zero or forward setpoint, or no command ever seen): healthy. A clock running backwards is not healthy.

    PRECONDITION: dock_drive must be the only writer on the command topic that can reverse. Any other writer that reverses (Nav2 BackUp at
    -0.1 to -0.15 m/s for about 3 s, teleop) is also "reversing" here and trips the rule after ``fix_max_age_s``, because no fix exists. Use the
    rule only in a stack like the dock stack (config/nav_watch_dock_fix.yaml).
    ``set_graph(ok, detail)``: the adapter reports a command topic whose publishers have another message type than expected: NOT healthy.
    """

    def __init__(self, fix_max_age_s: float = 1.5, reverse_threshold: float = 0.02, from_reverse_start: bool = True) -> None:
        for name, v in (("fix_max_age_s", fix_max_age_s), ("reverse_threshold", reverse_threshold)):
            if not (math.isfinite(v) and v > 0):
                raise ValueError(f"{name} must be positive and finite")
        self.fix_max_age_s, self.reverse_threshold = fix_max_age_s, reverse_threshold
        self.from_reverse_start = from_reverse_start
        self._graph_ok, self._graph_detail = True, ""
        self._fix_t: float | None = None
        self._reversing = False
        self._rev_since: float | None = None
        self._last_t: float | None = None

    def _seen(self, now: float) -> bool:
        if not math.isfinite(now):
            return False
        self._last_t = now if self._last_t is None else max(self._last_t, now)
        return True

    def set_graph(self, ok: bool, detail: str = "") -> None:
        self._graph_ok, self._graph_detail = bool(ok), detail

    def on_cmd(self, now: float, linear_x: float) -> None:
        if not self._seen(now):
            return
        rev = (not math.isfinite(linear_x)) or linear_x <= -self.reverse_threshold
        if rev and not self._reversing:
            self._rev_since = now
        if not rev:
            self._rev_since = None
        self._reversing = rev

    def on_fix(self, now: float) -> None:
        if self._seen(now):
            self._fix_t = now

    def ok(self, now: float) -> tuple[bool, str]:
        if not math.isfinite(now):
            return False, "non-finite time"
        if self._last_t is not None and now < self._last_t - 1e-9:
            return False, "clock ran backwards"
        if not self._graph_ok:
            return False, f"command topic not as expected: {self._graph_detail}"
        if not self._reversing:
            return True, ""
        ref = self._fix_t
        if self.from_reverse_start and self._rev_since is not None:
            ref = self._rev_since if ref is None else max(ref, self._rev_since)
        if ref is None:
            return False, "reversing, no dock fix ever seen"
        age = now - ref
        if age > self.fix_max_age_s + 1e-9:
            return False, f"reversing, newest dock fix (or reverse start) {age:.2f}s old > {self.fix_max_age_s:g}s"
        return True, ""


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
    chain: dict[str, Any] | None = None  # M8 command_chain: in_topic, out_topic, tick_topic, in_max_age_s, stop_grace_s
    reverse_fix: dict[str, Any] | None = None  # M8 reverse_fix: cmd_topic, fix_topic, fix_type, tick_topic, fix_max_age_s, ...

    @property
    def is_tf(self) -> bool:
        return self.tf_parent is not None

    @property
    def is_motion(self) -> bool:
        return self.motion is not None

    @property
    def is_chain(self) -> bool:
        return self.chain is not None

    @property
    def is_reverse_fix(self) -> bool:
        return self.reverse_fix is not None


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


TWIST_TYPE = "geometry_msgs/msg/Twist"
TWIST_TYPES = (TWIST_TYPE, "geometry_msgs/msg/TwistStamped")


def _twist_type(raw: Any, where: str) -> str:
    if raw not in TWIST_TYPES:
        raise ValueError(f"{where}: msg_type / cmd_type must be one of {TWIST_TYPES}")
    return raw


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
        motion = chain = reverse_fix = None
        cc, rf = w.get("command_chain"), w.get("reverse_fix")
        if cc is not None or rf is not None:
            if sum(x is not None for x in (cc, rf, mc)) > 1:
                raise ValueError(f"{where}: command_chain, reverse_fix and motion_consistency exclude each other")
            if tf is not None or "topic" in w or "type" in w:
                raise ValueError(f"{where}: command_chain / reverse_fix exclude topic/type/tf")
            spec_in = cc if cc is not None else rf
            kind = "command_chain" if cc is not None else "reverse_fix"
            need = ("in_topic", "out_topic") if cc is not None else ("cmd_topic", "fix_topic", "fix_type")
            if not isinstance(spec_in, dict) or not all(spec_in.get(k) for k in need):
                raise ValueError(f"{where}: {kind} needs {', '.join(need)}")
            if cc is not None:
                chain = {"in_topic": str(cc["in_topic"]), "out_topic": str(cc["out_topic"]),
                         "tick_topic": str(cc.get("tick_topic", "/odom")),
                         "in_max_age_s": _positive(cc.get("in_max_age_s", 0.35), where, "in_max_age_s"),
                         "stop_grace_s": _positive(cc.get("stop_grace_s", 1.0), where, "stop_grace_s"),
                         "msg_type": _twist_type(cc.get("msg_type", TWIST_TYPE), where),
                         "motion_linear": _positive(cc.get("motion_linear", 0.02), where, "motion_linear"),
                         "motion_angular": _positive(cc.get("motion_angular", 0.05), where, "motion_angular"),
                         "motion_grace_s": _positive(cc.get("motion_grace_s", 0.5), where, "motion_grace_s")}
                CommandChainGuard(chain["in_max_age_s"], chain["stop_grace_s"], chain["motion_linear"], chain["motion_angular"], chain["motion_grace_s"])
            else:
                reverse_fix = {"cmd_topic": str(rf["cmd_topic"]), "fix_topic": str(rf["fix_topic"]), "fix_type": str(rf["fix_type"]),
                               "tick_topic": str(rf.get("tick_topic", "/odom")),
                               "fix_max_age_s": _positive(rf.get("fix_max_age_s", 1.5), where, "fix_max_age_s"),
                               "reverse_threshold": _positive(rf.get("reverse_threshold", 0.02), where, "reverse_threshold"),
                               "from_reverse_start": bool(rf.get("from_reverse_start", True)),
                               "cmd_type": _twist_type(rf.get("cmd_type", TWIST_TYPE), where)}
                ReverseFixGuard(reverse_fix["fix_max_age_s"], reverse_fix["reverse_threshold"], reverse_fix["from_reverse_start"])
            topic, mtype, parent, child = (chain or reverse_fix)["tick_topic"], "nav_msgs/msg/Odometry", None, None
        elif mc is not None:
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
        specs.append(WatchSpec(cid, topic, mtype, _qos(w.get("qos"), where), hz, age, parent, child, mp, slack, motion, chain, reverse_fix))
    return specs
