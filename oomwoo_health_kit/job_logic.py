"""ROS-free logic for the job-following roster publisher (``job_roster_node``), an optional mechanism.

Fail-safe rules: start in the strictest set; switch to a stricter set at once and debounce only relaxation; unknown or
malformed state selects the strictest set; every set has one task_id and the same component ids (only ``critical``
and ``max_age_sec`` differ) because the aggregator restarts its arming window otherwise.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Iterable

# action_msgs/GoalStatus: ACCEPTED, EXECUTING, CANCELING are the active states.
ACTIVE_GOAL_STATUSES = frozenset({1, 2, 3})
DEFAULT_DEBOUNCE_S = 0.3
BACKWARD_JUMP_S = 0.25
DEFAULT_JOB_STATE_TOPIC = "oomwoo/health/job_state"
DEFAULT_SETTLE_S = 3.0
DEFAULT_PLUMBING_MAX_AGE_S = 0.5
PLUMBING_ID = "job_roster"
SOURCE_TYPES = ("static", "topic", "nav2_actions")


def any_active(statuses: Iterable[int]) -> bool:
    return any(s in ACTIVE_GOAL_STATUSES for s in statuses)


class Debouncer:
    """Stricter targets commit at once; a relaxation commits after being wanted continuously for ``hold_s``.

    ``ranks`` maps state -> strictness (higher is stricter). Equal ranks count as not relaxing. A clock that jumps backwards
    (between two calls) drops any pending relaxation and commits the strictest state: elapsed time can no longer be trusted.
    """

    def __init__(self, initial: str, hold_s: float, ranks: dict[str, float]) -> None:
        self.committed = initial
        self._hold = hold_s
        self._ranks = dict(ranks)
        self._strictest = max(self._ranks, key=lambda k: self._ranks[k])
        self._pending: tuple[str, float] | None = None
        self._last: float | None = None

    def _jumped_back(self, now: float) -> bool:
        # observe() and poll() are called with times taken a few ms apart (message callback vs timer): only a step back of more
        # than BACKWARD_JUMP_S counts as a clock jump
        back = self._last is not None and now < self._last - BACKWARD_JUMP_S
        self._last = now
        return back

    def _to_strictest(self) -> str | None:
        self._pending = None
        if self.committed == self._strictest:
            return None
        self.committed = self._strictest
        return self.committed

    def observe(self, now: float, desired: str) -> str | None:
        """Newly committed state if a stricter (or equal-rank) one was observed, else None."""
        if self._jumped_back(now):
            return self._to_strictest()
        if desired == self.committed:
            self._pending = None
            return None
        if self._ranks[desired] >= self._ranks[self.committed]:
            self.committed, self._pending = desired, None
            return desired
        if self._pending is None or self._pending[0] != desired:
            self._pending = (desired, now)
        return None

    def poll(self, now: float) -> str | None:
        """The newly committed relaxation (or, after a backward clock jump, the strictest state), or None."""
        if self._jumped_back(now):
            return self._to_strictest()
        if self._pending is None:
            return None
        state, since = self._pending
        if now - since >= self._hold:
            self.committed, self._pending = state, None
            return state
        return None


class StatusSource:
    """Combines action status topics into 'active' / 'inactive' / 'unknown'.

    A topic is *known* once it delivered a status message, or once it has had a publisher for ``settle_s`` without
    delivering one (its latched value would have arrived: no goal ever ran). Anything else is unknown, which the node
    maps to the strictest roster. ``valid`` is false while a status topic has no publisher (action server gone).
    """

    def __init__(self, topics: Iterable[str], settle_s: float = DEFAULT_SETTLE_S) -> None:
        self.topics = tuple(topics)
        self.settle_s = settle_s
        self._active: dict[str, bool] = {}
        self._pub_since: dict[str, float | None] = {t: None for t in self.topics}
        self._count: dict[str, int] = {t: 0 for t in self.topics}

    def on_status(self, topic: str, active: bool) -> None:
        self._active[topic] = active

    def on_publishers(self, now: float, topic: str, count: int) -> None:
        self._count[topic] = count
        if count <= 0:
            self._pub_since[topic] = None
        elif self._pub_since[topic] is None:
            self._pub_since[topic] = now

    def valid(self) -> bool:
        return all(c > 0 for c in self._count.values())

    def state(self, now: float) -> str:
        if any(self._active.values()):
            return "active"
        for t in self.topics:
            since = self._pub_since[t]
            if t not in self._active and (since is None or now - since < self.settle_s):
                return "unknown"
        return "inactive"


@dataclass(frozen=True)
class RosterSet:
    task_id: str
    components: tuple[dict[str, Any], ...]

    def payload(self) -> dict[str, Any]:
        return {"task_id": self.task_id, "components": [dict(c) for c in self.components]}

    @property
    def strictness(self) -> tuple[int, float]:
        crit = [c for c in self.components if c["critical"]]
        return len(crit), -sum(c["max_age_sec"] for c in crit)


@dataclass(frozen=True)
class JobConfig:
    rosters: dict[str, RosterSet]
    strictest: str
    ranks: dict[str, float]
    debounce_s: float
    source_type: str
    job_state_topic: str
    status_topics: tuple[str, ...]
    active_state: str
    inactive_state: str
    settle_s: float
    plumbing_max_age_s: float
    allow_arming_restart: bool


def _num(raw: Any, where: str, low: float, strict: bool = False) -> float:
    try:
        v = float(raw)
    except (TypeError, ValueError):
        raise ValueError(f"{where} must be a number") from None
    if not math.isfinite(v) or v < low or (strict and v <= low):
        raise ValueError(f"{where} must be {'>' if strict else '>='} {low:g} and finite")
    return v


def _roster(name: str, raw: Any, plumbing: float | None) -> RosterSet:
    if not isinstance(raw, dict) or not isinstance(raw.get("task_id"), str) or not raw["task_id"]:
        raise ValueError(f"roster {name!r}: task_id is required")
    comps = raw.get("components")
    if not isinstance(comps, list) or not comps:
        raise ValueError(f"roster {name!r}: components must be a non-empty list")
    out, ids = [], set()
    for c in comps:
        if not isinstance(c, dict) or not isinstance(c.get("component_id"), str) or not c["component_id"]:
            raise ValueError(f"roster {name!r}: every component needs a component_id")
        cid = c["component_id"]
        if cid in ids:
            raise ValueError(f"roster {name!r}: duplicate component {cid!r}")
        ids.add(cid)
        crit = c.get("critical", True)
        if not isinstance(crit, bool):
            raise ValueError(f"roster {name!r}/{cid}: critical must be true or false")
        if cid == PLUMBING_ID and not crit:
            raise ValueError(f"roster {name!r}: {PLUMBING_ID} must be critical in every set")
        if "max_age_sec" not in c:
            raise ValueError(f"roster {name!r}/{cid}: max_age_sec is required")
        age = _num(c["max_age_sec"], f"roster {name!r}/{cid}: max_age_sec", 0.0, strict=True)
        out.append({"component_id": cid, "critical": crit, "max_age_sec": age})
    if plumbing is not None and PLUMBING_ID not in ids:
        out.append({"component_id": PLUMBING_ID, "critical": True, "max_age_sec": plumbing})
    return RosterSet(raw["task_id"], tuple(out))


def parse_job_config(doc: Any) -> JobConfig:
    if not isinstance(doc, dict) or not isinstance(doc.get("rosters"), dict) or not doc["rosters"]:
        raise ValueError("config needs a non-empty 'rosters' mapping")
    src = doc.get("state_source")
    if not isinstance(src, dict) or src.get("type") not in SOURCE_TYPES:
        raise ValueError(f"state_source.type must be one of {SOURCE_TYPES}")
    stype = src["type"]
    plumbing_age = _num(doc.get("plumbing_max_age_s", DEFAULT_PLUMBING_MAX_AGE_S), "plumbing_max_age_s", 0.0, True)
    rosters = {
        str(k): _roster(str(k), v, None if stype == "static" else plumbing_age) for k, v in doc["rosters"].items()
    }
    if stype == "static" and len(rosters) != 1:
        raise ValueError("a static state_source needs exactly one roster set")
    allow = doc.get("allow_arming_restart", False)
    if not isinstance(allow, bool):
        raise ValueError("allow_arming_restart must be true or false")
    ref = next(iter(rosters.values()))
    if not allow:
        for name, rs in rosters.items():
            if rs.task_id != ref.task_id or {c["component_id"] for c in rs.components} != {
                c["component_id"] for c in ref.components
            }:
                raise ValueError(
                    f"roster {name!r}: task_id and component ids must be identical in every set (a change restarts "
                    "the aggregator's arming window and soft-stops the robot); only critical/max_age_sec may differ, "
                    "or set allow_arming_restart: true"
                )
    strictest = max(rosters, key=lambda n: rosters[n].strictness)  # first wins on ties
    if "initial_state" in doc and str(doc["initial_state"]) != strictest:
        raise ValueError(f"initial_state must be the strictest set ({strictest!r}); it is not configurable")
    keys = sorted({rs.strictness for rs in rosters.values()})
    ranks = {n: float(keys.index(rs.strictness)) for n, rs in rosters.items()}
    debounce = _num(doc.get("debounce_s", DEFAULT_DEBOUNCE_S), "debounce_s", 0.0)
    topics: tuple[str, ...] = ()
    active, inactive = str(src.get("active_state", "driving")), str(src.get("inactive_state", "idle"))
    if stype == "nav2_actions":
        t = src.get("status_topics")
        if not isinstance(t, list) or not t or not all(isinstance(x, str) and x for x in t):
            raise ValueError("state_source.status_topics must be a non-empty list of topic names")
        topics = tuple(t)
        for s in (active, inactive):
            if s not in rosters:
                raise ValueError(f"state_source state {s!r} is not a roster set")
    return JobConfig(
        rosters, strictest, ranks, debounce, stype, str(src.get("topic", DEFAULT_JOB_STATE_TOPIC)), topics,
        active, inactive, _num(src.get("settle_s", DEFAULT_SETTLE_S), "settle_s", 0.0), plumbing_age, allow,
    )
