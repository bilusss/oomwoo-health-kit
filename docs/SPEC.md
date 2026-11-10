# oomwoo-health-kit — spec

Verification kit for the OOMWOO `health-monitor` RFC
(`makerspet/oomwoo/contributions/health-monitor`).

## Why this and not another aggregator

A candidate aggregator already exists (`xbattlax/health-monitor`, v0.1.0). Read-only
audit on 2026-11-09 found its logic sound but unproven: it is tested only against a
faked `rclpy`, nothing consumes its MCU heartbeat, no node emits real heartbeats, and
the RFC criteria "stops the robot in sim" and "no false trips under load / jitter" are
not demonstrated. This kit supplies the missing half and is aggregator-agnostic: it
speaks the topics and JSON payloads of the published contract
(`docs/SOFTWARE_INTERFACES.md` "Stack Health Monitor Draft" and
`xbattlax/health-monitor/docs/stack_watchdog_contract.md`), so it can score any
candidate, including a later one of our own.

## Deliverables

| # | Piece | Needs ROS | Milestone |
|---|-------|-----------|-----------|
| 1 | MCU stand-in core: watchdog state machine + `/cmd_vel` gate | no | M1 |
| 2 | Work-path heartbeat helper core (sequence, level, stamp) | no | M1 |
| 3 | `rclpy` adapters + launch files for 1 and 2 | yes | M2 |
| 4 | Fault-injection launch tests on real DDS: kill, hang (SIGSTOP), never-started, refuse-to-arm | yes | M2 |
| 5 | Jitter / load / sim-clock false-trip tests; detect-to-stop latency and distance measured in Gazebo headless | yes | M3 |
| 6 | Findings report + recommended timing table for SOFTWARE_INTERFACES.md | — | M3 |

## M1 design

Layout mirrors the existing OOMWOO packages: `ament_python`, Apache-2.0, pure-Python
core with no ROS imports, thin adapters added in M2.

### MCU stand-in core (`oomwoo_health_kit/mcu_standin.py`)

Models what the real MCU firmware will do with the aggregate heartbeat. Time is always
passed in by the caller (monotonic seconds) — the core never reads a clock.

States:

- `UNARMED` — no heartbeat seen since start. Motion inhibited.
- `RUN` — heartbeats fresh. Commands pass through unchanged.
- `SOFT_STOP` — last heartbeat older than `soft_stop_timeout_s`. Motion inhibited;
  recoverable.
- `CPU_RESET` — last heartbeat older than `cpu_reset_timeout_s`. Motion inhibited;
  reset line asserted (reported once per entry).

Rules:

- Fail-safe: every state except `RUN` outputs zero velocity.
- Entering `RUN` from any other state requires `rearm_count` consecutive heartbeats,
  each arriving within `soft_stop_timeout_s` of the previous one. A single stray
  heartbeat never re-enables motion.
- A heartbeat whose time is earlier than the previous one is ignored and counted.
- The core records every transition as (time, from, to, reason) and exposes counters:
  heartbeats received, ignored, soft stops, resets.
- Non-finite time (NaN, +/-inf) passed to any method is counted in `invalid_time`, changes no state, gives no streak credit, and makes `gate` return zero.
- The latest finite time seen across `heartbeat`, `update` and `gate` is tracked; an older `now` is stale: heartbeat is ignored, update evaluates at the latest time, gate returns zero.
- The heartbeat streak resets to 0 on entering any non-RUN state, so re-arming always needs `rearm_count` heartbeats arriving after the stop.
- `reset_asserted` is true while in `CPU_RESET`; `Status.reset_asserted_now` is the one-shot form.
- Defaults, all constructor parameters: `soft_stop_timeout_s=0.15`,
  `cpu_reset_timeout_s=300.0`, `rearm_count=3`. Validate
  `0 < soft_stop_timeout_s < cpu_reset_timeout_s` and `rearm_count >= 1`.

The timeouts are experimental simulation settings. They follow the firmware bring-up policy agreed in
makerspet/oomwoo-firmware#1: an MCU-local heartbeat safe-stop deadline of 150 ms, no CPU reset on a short
heartbeat gap, and CPU-reset supervision as a separate mechanism with an initial timeout of about five minutes.
The real cutoff and reset policy belongs to the firmware / IO-board contract, not to this kit. The stand-in
measures time on the ROS clock (or the host steady clock) as a simulation convenience; that is not equivalent to
the MCU's own clock. Until M4 the defaults were 0.2 s / 2.0 s; M3 numbers were taken with those.

### Heartbeat helper core (`oomwoo_health_kit/heartbeat.py`)

Makes the RFC's "heartbeat from the work path, not from a timer" rule the easy path.

- `WorkHeartbeat(component_id)` with `beat(now, level="ok", detail="")` returning the
  payload dict to publish. The node calls `beat` at the point where real work
  completed (control cycle done, scan produced).
- Payload field names and health-level vocabulary must match the published contract
  exactly; the contract file is the authority, not this spec.
- `sequence` increases by one per beat, starting at 1 (the contract does not define a start value; 1 matches the reference node), so a
  consumer can tell a live producer from a replayed message.
- `to_json(payload)` gives the string for the `std_msgs/String` transport.
- No timer, no thread, no clock read inside the helper.

### Tests (M1)

`pytest`, no ROS. Cover every state transition of the stand-in including the timeout
boundaries, the re-arm rule (stray heartbeat, broken streak), out-of-order time,
parameter validation, and the gate output in each state. For the helper: payload
schema against the contract, sequence monotonicity, JSON round-trip.

## Out of scope

Battery, charging, motor-driver or any real MCU firmware; changes to upstream repos;
publishing anything without the owner's approval.

## M2 design (as built)

- `tools/dev.sh {up|build|test|shell|exec|down}` drives the `ohk-dev` container (image
  `makerspet/oomwoo:jazzy-dev`) as the host uid, so the project has no root files; build,
  install and log live in the `ohk-ws` volume at `/ws`. Aggregator cloned there; `OHK_AGG_REF` selects the ref (default `b2b9796`, PR heads are fetched) and `build` rebuilds the aggregator when it changes.
- `qos.py`: roster/stack/`mcu_state` reliable, transient-local, depth 1; component and MCU
  heartbeats reliable, volatile, depth 10 (contract names no depth; matches the aggregator).
- `mcu_standin_node`: `oomwoo/health/mcu_heartbeat` + `cmd_vel_in` in, `cmd_vel` +
  `oomwoo/health/mcu_state` (JSON, one message per core transition) out; 100 Hz update timer;
  zero Twist on every non-RUN entry. `clock_source` = `steady` (default) or `ros`; with `ros`,
  a clock frozen for > soft stop of steady time forces zero and sets `clock_fault` (adapter only).
  Twist maps to core `Velocity` as linear.x / angular.z.
- `RosHeartbeat` wraps `WorkHeartbeat`. `fake_component_node` beats after each work cycle;
  SIGUSR1 stalls the work loop (process and executor stay alive); `beat_from_timer:=true` is
  the RFC anti-pattern, and test (i) documents that failure as undetectable.
- `roster_publisher_node`: one latched roster. Aggregator `max_age_sec` and MCU soft stop add
  in series: worst case about max_age + soft_stop + 10 ms tick + DDS latency. The aggregator
  default 0.5 s still stops the robot, later (about 0.71 s with the earlier 0.2 s soft stop, about 0.67 s with
  0.15 s; computed); 0.15 s keeps it under 0.4 s (launch-test bound: max_age + soft stop + one tick + 0.1 s margin).
- `test/launch/`: scenarios a-j, each a real multi-process launch on its own `ROS_DOMAIN_ID`
  (j: frozen `/clock`, stand-in only). Latencies (signal -> first zero Twist) go to
  `$OHK_RESULTS`; re-arm time is quantised to the 20 Hz probe. Host pytest skips `test/launch`.

## M4 (as built)

- Stand-in defaults aligned with the firmware policy above (0.15 s soft stop, 300 s CPU reset). Launch test f passes
  an explicit 2.0 s reset timeout so it still exercises the reset path; its soft-stop bound is soft stop + one tick +
  50 ms, and the b/c/h bound is max_age + soft stop + one tick + 100 ms.
- `bench/soak.py`: unattended long run without Gazebo (cells: idle / cpu / mixed load, max_age 0.10 / 0.15 / 0.25 s,
  20 / 50 Hz; a 10 min steady window and 40 kill/restart cycles each), one JSON line per cell in `results/soak.jsonl`,
  quiet windows, `--smoke`. Start it detached with `tools/dev.sh soak-start`, stop it with `tools/dev.sh soak-stop`.
