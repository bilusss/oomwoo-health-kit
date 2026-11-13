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
- `qos.py`: roster/stack/`mcu_state` reliable, transient-local, depth 1; component heartbeats reliable, volatile,
  depth 10; MCU heartbeat reliable, volatile, depth 1 (history depths as in the upstream contract, see M5).
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
- `test/launch/`: scenarios a-k, each a real multi-process launch on its own `ROS_DOMAIN_ID`
  (j: frozen `/clock`, stand-in only; k: frozen `/clock`, aggregator). Latencies (signal -> first zero Twist) go to
  `$OHK_RESULTS`; re-arm time is quantised to the 20 Hz probe. Host pytest skips `test/launch`.

## M4 (as built)

- Stand-in defaults aligned with the firmware policy above (0.15 s soft stop, 300 s CPU reset). Launch test f passes
  an explicit 2.0 s reset timeout so it still exercises the reset path; its soft-stop bound is soft stop + one tick +
  50 ms, and the b/c/h bound is max_age + soft stop + one tick + 100 ms.
- `bench/soak.py`: unattended long run without Gazebo (cells: idle / cpu / mixed load, max_age 0.10 / 0.15 / 0.25 s,
  20 / 50 Hz; a 10 min steady window and 40 kill/restart cycles each), one JSON line per cell in `results/soak.jsonl`,
  quiet windows, `--smoke`. Start it detached with `tools/dev.sh soak-start`, stop it with `tools/dev.sh soak-stop`.
- Soak hygiene: container created with `--shm-size=1g` (recreated if smaller), stale Fast DDS shm removed before every
  stack start, cell invalid if /dev/shm free < 64 MiB; steady windows are valid only if the stand-in ends in RUN and
  heartbeat and cmd_vel counts are >= 95% of nominal; kills are skipped (and counted) when the system is not steady;
  children die with `soak.py` (PR_SET_PDEATHSIG); `soak-start` refuses a second instance.

## M5 (as built, 2026-11-11)

- Soak transport fix: Fast DDS reads `FASTRTPS_DEFAULT_PROFILES_FILE` once per process, so the long-lived probe in
  `soak.py` kept default transports in `udp_only` / `shm_hc60` cells. Each cell now runs in a fresh child
  (`soak.py --worker`, environment set before rclpy is imported); the parent never imports rclpy. JSONL format, quiet
  windows, lock and cleanup are unchanged, and the strict check stays (`udp_only` with any `fastrtps_*` file in
  `/dev/shm` is invalid). Smoke: `udp_only` shows 0 files and is valid.
- Contract alignment (upstream `SOFTWARE_INTERFACES.md`, "Stack Health Monitor Draft"): history depths roster/stack 1,
  component heartbeat 10, MCU heartbeat 1. `qos.mcu_heartbeat_qos()` (depth 1) is used by the stand-in subscriber, the
  probes and test j. Sequences start at 1 in both kit and aggregator. Open differences, kit side: the stand-in does not
  drop an MCU heartbeat by `stamp_sec` age (bridge rule in the contract; the stand-in judges arrival only). Aggregator
  side: it publishes the MCU heartbeat with depth 10 (contract: 1); at `b2b9796` it has no clock guard.
- Test k (`test_k_frozen_clock_aggregator.py`): all nodes on `use_sim_time`, stand-in `clock_source=steady`, `/clock`
  stops after RUN. Components keep beating on wall time with frozen stamps. Result on `b2b9796` and on the PR #2 head
  alike: no MCU heartbeat and no stack message after the freeze (the aggregator's ROS timer does not fire without
  `/clock`, so the PR's `clock_stalled` guard is never reached), the robot stops. The last latched stack message stays
  `healthy`. `EXPECTED_FAIL` in the test lists refs known to violate the contract (empty now).

## M6a design (as built, 2026-11-13, revised after review)

Heartbeat sources for nodes that cannot be modified (Nav2, slam_toolbox, ros_gz_bridge), made by observing their outputs.
Tests l-t use fake publishers; M6c ran the watcher, the static roster and the fault set on Gazebo + Nav2 + slam_toolbox (see M6c).
Nothing here has run on hardware, with AMCL, docking, or any other job than the NavigateToPose loop.

- **Default: a static roster** (`config/nav_rosters.yaml`, `job_roster` with `state_source: static`): critical = lidar, odom,
  localization, `drive_supervision`; advisory = costmaps. Why: with "hold last setpoint" MCU semantics a robot that moves while a
  job-following roster says idle is unsupervised, and docking, wall-follow, coverage, recovery and teleop publish cmd_vel outside
  the Nav2 actions. Supervising by motion needs no job state. Plumbing: if `topic_heartbeat` dies every component goes stale.
- `topic_heartbeat` (`topic_heartbeat_node.py`, logic in ROS-free `topic_watch.py`): YAML watches: `topic`+`type`, `tf`, or
  `motion_consistency` (odom + the command topic the MCU receives; tracks the HELD SETPOINT, the last command value. Beat only if
  a command newer than `cmd_max_age_s` exists, OR the held setpoint is zero (or none ever seen) and odom was under the thresholds
  continuously for `settle_s` (0.1) or less than `coast_max_s` (1.0 default) passed since the setpoint became zero. A non-zero
  held setpoint with no fresh command never beats, whatever odom says (held creep, stalled wheels with live odometry). Stale or
  non-finite odometry or command values give none. The command subscription is reliable.) Heartbeats come from subscription callbacks, never
  a timer: when every publisher on the topic is silent and no queued messages remain, the heartbeats stop. Fail-open guards: more
  than `max_publishers` (default 1) writers -> no heartbeat (count cached 0.5 s, refreshed from callbacks); a message older than
  1/max_emit_hz + `queue_slack_s` by the publisher's SOURCE timestamp is dropped (the receive stamp is never used: Fast DDS takes
  it at take time). Zero source stamp -> header stamp (node clock must be the stamp clock); neither -> guard unavailable, logged
  once per watch and noted in the heartbeat detail. Assumes publisher and watcher share one host clock. A stalled watcher
  cannot replay queued messages; `tf` watches have NO writer guard (/tf has many writers by design); `transient_local` refused; with `stamp_max_age_s` a missing, zero, old or
  future header stamp is `degraded` (unhealthy for a critical component in the aggregator).
- `job_roster` (optional job-following mechanism, `config/nav_rosters_job.yaml`): loader refuses sets with different task_id or ids
  (the aggregator restarts its 0.25 s arming window otherwise) unless `allow_arming_restart`; starts in the strictest set; stricter
  set at once, only relaxation debounced (0.3 s); unknown state -> strictest; String/status subscriptions transient-local.
  Plumbing liveness: component `job_roster` (critical everywhere) beats at 10 Hz from the 20 ms re-validation loop, `degraded`
  if an action status topic has no matched publisher or the posted roster differs from the committed state; a dead node stops
  the robot after max_age 0.5 s + soft stop. The loop is a timer by design: it proves the node runs and the status topics still
  have a MATCHED publisher endpoint, not that the job state is right nor that the action server is alive: a SIGKILLed server stays
  matched for its DDS lease (about 20 s), a SIGSTOPped one indefinitely. Dead or frozen Nav2 is caught by drive_supervision. A status topic counts as "no goal" only after `settle_s` (3 s) with a publisher but no latched message
  (action servers publish no initial status); a restart mid-goal is fail-open if the latched status takes longer than that.
- Aggregator (b2b9796): a roster whose task_id/id set changes restarts arming (0.3-0.5 s of zero cmd_vel); `degraded` is not in
  {ok, healthy}, so it is `unhealthy_critical` on a critical component. Nav2 is silent while planning, so a job set that makes
  drive_path critical at goal start faults at once; the static roster does not.
- Honesty limits: an external watcher proves that a node produces output, not that it is correct (a node republishing stale data,
  a localization stuck on a wrong pose, a controller emitting wrong commands pass), and cannot see a node that is silent by design
  while idle. drive_supervision cannot tell a legitimate command from a wrong one, nor that the wheels follow it
  (it reads ground-truth odometry in the sim); a non-zero setpoint left by a dead commander keeps the stack stopped until a
  command arrives, even after the robot has halted. A backward clock jump (more than 0.25 s) makes the
  job roster select the strictest set. Numbers come from fake publishers on a loaded host, single runs; latencies measured
  from the signal depend on message phase: worst case = wait for next message (<= source period) + max_age + aggregator tick 0.05
  + soft stop 0.15 + stand-in tick 0.01, the tests add 0.1 s margin.
- Tests l-t (`test/launch`, fakes in `ohk_fakes.py`); results JSON holds the measured values.

## M6c (as built, 2026-11-13): watcher + static roster on Gazebo/Nav2/slam_toolbox
All numbers: RTF 1.0, software GL, 1-4 containers in parallel, single runs unless a count is given. Results in `results/fullstack/`.
- `health_sources:=watcher` = `job_roster` (static, `config/nav_rosters.yaml`) + `topic_heartbeat` (`config/nav_watch_sim.yaml`). Verified graph:
  /scan 5.0 Hz, /odom 50 Hz, map->odom 5 Hz distinct stamps (slam_toolbox republishes the stamp at 50 Hz, stamp ~0.19 s AHEAD of sim now;
  /pose is silent when stationary), costmap footprints 5 Hz (both; M6_RECON's 2 Hz was wrong), all single writer, same driving/stationary.
  One ros_gz_bridge process carries /scan, /odom, /tf, /clock, /joint_states and takes cmd_vel_gated. /cmd_vel (stand-in input) has one
  writer here (collision_monitor), /cmd_vel_nav six. drive_supervision watches /cmd_vel: any fresh command counts, it proves that a
  fresh command exists while a setpoint is held or the robot moves, not that it is correct. max_age: lidar 0.6, odom 0.15,
  localization 0.6 (measured max gap 0.22 s), drive_supervision 0.15, local_costmap 0.6 (critical: the collision monitor is
  disabled in this stack, the local costmap is the only obstacle protection), global_costmap 0.6 (advisory).
- Clock: health nodes (aggregator, watcher, stand-in) on WALL time (`--health-clock wall`, recorded as `health_clock`). The aggregator compares
  heartbeat stamps with its own clock, so all must share one; sim time costs (1 run, 3 containers) topic_heartbeat 0.50 + health_monitor
  0.32 + stand-in 0.31 + job_roster 0.23 core vs 0.17 + 0.04 + small on wall, and a frozen /clock freezes the aggregator's
  staleness check. Consequence: header stamps (sim) are not checked (no `stamp_max_age_s`); freshness is receive-side (source timestamp
  age, strictly newer tf stamps). Source timestamps are non-zero wall-clock for all watched topics (bridge, Nav2, slam_toolbox).
- Goal start/end (new rule, final config, 260 goals incl. ~64 cancelled and 20 spin/backup/wait probes: 0 per goal): 0 aggregator faults,
  0 soft stops in every phase. Motion without a command newer than 0.15 s: 0 events (Nav2 keeps commanding until the robot is below
  0.02 m/s); Gazebo coast after a zero command about 0.12 s, hence `coast_max_s` 0.5 in the sim config. An unplannable goal aborts in
  0.01 s and starts no BT recovery; recoveries were exercised by direct spin/backup/wait actions, not BT-triggered. Pre-fix (old
  stationary rule) 104 goals also gave 0.
- Faults (5 each, `faults.jsonl`): SIGSTOP Nav2 container 5 s, SIGKILL Nav2, SIGKILL slam, SIGKILL the bridge, Gazebo pause 5 s. Zero gated
  command 0.22-0.72 s after the signal, standstill 0.46-0.84 s (0.08-0.15 m), except the bridge: the dead last hop cannot stop the robot, it
  drove 1.7-2.1 m for 12-13 s until it stopped on its own (health loop and stand-in did their part at 0.22-0.28 s). A dead commander's
  non-zero setpoint keeps the stack stopped until a command arrives (SIGKILL cases never recover; SIGSTOP/pause recover ~8 s after resume).
- N4: SIGKILL + instant respawn of the bridge keeps the guard (2 writers) down 20.0-20.5 s (DDS lease), stand-in RUN again at 20.8 s;
  fail-safe, no mitigation in rclpy. Not measured: other jobs, AMCL, hardware, runs over 15 min, GPU.
