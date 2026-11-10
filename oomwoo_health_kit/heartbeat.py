"""Work-path heartbeat helper: builds contract payloads, owns no timer or clock."""

from __future__ import annotations

import json
import math
from typing import Any

HEALTH_LEVELS = frozenset({"ok", "healthy", "warn", "degraded", "error"})
INITIAL_SEQUENCE = 1


class WorkHeartbeat:
    """Produces component heartbeat payloads for /oomwoo/health/component."""

    def __init__(self, component_id: str) -> None:
        if not component_id:
            raise ValueError("component_id is required")
        self.component_id = component_id
        self._next_sequence = INITIAL_SEQUENCE

    def beat(self, now: float, level: str = "ok", detail: str = "") -> dict[str, Any]:
        """Return the payload for one completed unit of work.

        ``now`` must come from the same ROS clock as the monitor.
        """
        if level not in HEALTH_LEVELS:
            raise ValueError(f"unknown health level: {level!r}")
        if not math.isfinite(now):
            raise ValueError("now must be finite")
        payload = {
            "component_id": self.component_id,
            "health": level,
            "stamp_sec": float(now),
            "sequence": self._next_sequence,
            "detail": detail,
        }
        self._next_sequence += 1
        return payload


def to_json(payload: dict[str, Any]) -> str:
    """Serialise a payload for the std_msgs/String transport."""
    return json.dumps(payload, sort_keys=True)
