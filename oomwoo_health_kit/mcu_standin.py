"""MCU stand-in: watchdog state machine and /cmd_vel gate.

Time is always supplied by the caller in monotonic seconds; nothing here reads a clock.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum


class McuState(Enum):
    UNARMED = "unarmed"
    RUN = "run"
    SOFT_STOP = "soft_stop"
    CPU_RESET = "cpu_reset"


@dataclass(frozen=True)
class Velocity:
    linear: float = 0.0
    angular: float = 0.0


ZERO_VELOCITY = Velocity()


@dataclass(frozen=True)
class Transition:
    time: float
    from_state: McuState
    to_state: McuState
    reason: str


@dataclass(frozen=True)
class Status:
    """Result of evaluating the stand-in at one instant."""

    state: McuState
    reset_asserted_now: bool = False


class McuStandin:
    """Watchdog over the aggregate heartbeat; inhibits motion unless healthy."""

    def __init__(
        self,
        soft_stop_timeout_s: float = 0.2,
        cpu_reset_timeout_s: float = 2.0,
        rearm_count: int = 3,
    ) -> None:
        if not 0.0 < soft_stop_timeout_s < cpu_reset_timeout_s:
            raise ValueError("require 0 < soft_stop_timeout_s < cpu_reset_timeout_s")
        if rearm_count < 1:
            raise ValueError("rearm_count must be >= 1")
        self.soft_stop_timeout_s = soft_stop_timeout_s
        self.cpu_reset_timeout_s = cpu_reset_timeout_s
        self.rearm_count = rearm_count

        self.state = McuState.UNARMED
        self.transitions: list[Transition] = []
        self.heartbeats_received = 0
        self.heartbeats_ignored = 0
        self.soft_stops = 0
        self.resets = 0
        self.invalid_time = 0

        self._last_heartbeat: float | None = None
        self._latest: float | None = None
        self._streak = 0

    @property
    def reset_asserted(self) -> bool:
        return self.state is McuState.CPU_RESET

    def update(self, now: float) -> Status:
        """Apply timeouts at ``now``; reset is flagged only on the entering call.

        Non-finite times are counted and ignored; times older than the latest
        seen are evaluated at the latest seen time instead.
        """
        if not math.isfinite(now):
            self.invalid_time += 1
            return Status(self.state)
        effective = now if self._latest is None else max(now, self._latest)
        self._latest = effective
        return self._evaluate(effective)

    def heartbeat(self, now: float) -> Status:
        """Register a heartbeat received at ``now``."""
        if not math.isfinite(now):
            self.invalid_time += 1
            return Status(self.state)
        if self._latest is not None and now < self._latest:
            self.heartbeats_ignored += 1
            return Status(self.state)

        self._latest = now
        status = self._evaluate(now)
        self.heartbeats_received += 1
        within = (
            self._last_heartbeat is not None
            and now - self._last_heartbeat <= self.soft_stop_timeout_s
        )
        self._streak = self._streak + 1 if within else 1
        self._last_heartbeat = now

        if self.state is not McuState.RUN and self._streak >= self.rearm_count:
            self._enter(now, McuState.RUN, f"{self._streak} consecutive heartbeats")
        return Status(self.state, status.reset_asserted_now)

    def gate(self, now: float, command: Velocity) -> Velocity:
        """Pass ``command`` only in RUN; otherwise (or on a bad time) output zero."""
        if not math.isfinite(now):
            self.invalid_time += 1
            return ZERO_VELOCITY
        if self._latest is not None and now < self._latest:
            return ZERO_VELOCITY
        if self.update(now).state is McuState.RUN:
            return command
        return ZERO_VELOCITY

    def _evaluate(self, now: float) -> Status:
        entered_reset = False
        if self._last_heartbeat is not None and self.state in (
            McuState.RUN,
            McuState.SOFT_STOP,
        ):
            age = now - self._last_heartbeat
            if age > self.cpu_reset_timeout_s:
                self._enter(now, McuState.CPU_RESET, f"no heartbeat for {age:.3f}s")
                entered_reset = True
            elif age > self.soft_stop_timeout_s and self.state is McuState.RUN:
                self._enter(now, McuState.SOFT_STOP, f"no heartbeat for {age:.3f}s")
        return Status(self.state, entered_reset)

    def _enter(self, now: float, new: McuState, reason: str) -> None:
        self.transitions.append(Transition(now, self.state, new, reason))
        self.state = new
        if new is not McuState.RUN:
            self._streak = 0
        if new is McuState.SOFT_STOP:
            self.soft_stops += 1
        elif new is McuState.CPU_RESET:
            self.resets += 1
