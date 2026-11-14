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

## M6c (as built, 2026-11-13, fixed 2026-11-14): watcher + static roster on Gazebo/Nav2/slam_toolbox
RTF 1.0, software GL, 1-4 containers in parallel, single runs unless counted. Results in `results/fullstack/` (first pass kit 8d7211e in `_m6c_first_pass_*`). New-code sessions are labelled `8d7211e-dirty` (uncommitted fixes).
- `health_sources:=watcher` = `job_roster` (static, `config/nav_rosters.yaml`) + `topic_heartbeat` (`config/nav_watch_sim.yaml`). Verified graph:
  /scan 5.0 Hz, /odom 50 Hz, map->odom 5 Hz distinct stamps (the stamp is republished at 50 Hz, ~0.19 s ahead of sim now; /pose is silent
  when stationary), costmap footprints 5 Hz, all single writer, same driving and stationary. One ros_gz_bridge process carries /scan, /odom, /tf, /clock
  and takes cmd_vel_gated. drive_supervision watches /cmd_vel (stand-in input, 1 writer: collision_monitor; /cmd_vel_nav has 6): a fresh NON-ZERO command, or a
  zero setpoint with odom settled / inside `coast_max_s`; zero commands give no freshness credit and do not extend the coast window (it starts when the setpoint
  first becomes zero, cleared by a non-zero command); a queued-too-long command still updates the held setpoint. max_age: lidar 0.6, odom 0.15, localization 0.6,
  drive_supervision 0.15, local_costmap 0.6 (critical), global_costmap 0.6 (advisory).
- Clock: health nodes on WALL time (aggregator compares heartbeat stamps with its own clock; sim time cost 1.3 core vs 0.3, one run; a frozen /clock would freeze the
  staleness check). Header-stamp checks are therefore OFF (stamps are sim time); freshness is receive-side (source timestamp age, strictly newer tf stamps).
- Limits of the watches: local_costmap's footprint shows that its update loop runs, not that obstacle data is fresh (it stayed alive with /scan dead). The "no command
  ever seen" branch is safe only because the stand-in never restores an old setpoint after a soft stop; firmware must match. coast_max_s 0.5 is justified only for this
  sim at 0.2 m/s (Nav2 flows: no uncommanded motion; Gazebo coast after a zero command ~0.12 s).
- Goal accounting, first pass (kit cc5691e-dirty, 260 = 220 NavigateToPose: 156 succeeded, 60 cancelled mid-drive, 4 cut by session end; 20 unplannable; 20 direct recovery
  probes; 104 of them with coast_max_s 1.0): 0 episodes, 0 soft stops. New code (8d7211e-dirty, 4 sessions x 700 sim s, 228 NavigateToPose: 148 succeeded, 36 cancelled mid-drive,
  20 cancelled early (1 s), 4 cut by session end, 20 unplannable (aborted in 0.01 s, no motion); plus 20 spin/backup/wait probes, not BT-triggered): 0 non-emitting episodes,
  0 soft stops in any phase, stand-in counter delta 0, aggregator healthy_for_sec 710 s at the end of every session.
- Faults, 5 each (`faults.jsonl`; seconds from the signal): zero gated command / standstill / distance. SIGSTOP Nav2 5 s: 0.37-0.41 / 0.48-0.51 / 0.08-0.09 m, blamed
  drive_supervision + local_costmap, 4/5 recovered 7.7-8.3 s after SIGCONT (1/5 did not within 25 s: local_costmap went stale again; cause not determined). SIGKILL Nav2:
  0.35-0.41 / 0.46-0.53 / 0.08-0.09 m, no recovery. SIGKILL slam: 0.51-0.71 / 0.61-0.83 / 0.11-0.15 m (localization, local_costmap). SIGKILL watcher: 0.25-0.29 / 0.37-0.40 /
  0.06 m (all components stale). SIGKILL aggregator: 0.11-0.15 / 0.23-0.26 / 0.03-0.04 m (no stack message; the stand-in's own timeout). Gazebo pause 5 s: fault instant = last
  advance of /clock; aggregator fault 0.04-0.17 s, zero gated 0.14-0.27 s; physics is frozen so standstill is not a stopping measure; commands resumed 0.8-9.0 s after unpause.
- SIGKILL of ros_gz_bridge: the gate sits before the bridge, the only path to the simulated motors. The loop detected the kill (fault 0.13-0.17 s, zero command 0.24-0.28 s) but could
  not deliver the zero; the diff-drive kept its last velocity for 12-15 s (1.7-2.1 m). In 5/5 it ended at x=-2.276, heading pi, 0.12 m from the first occupied cell of the west wall,
  speed decaying over ~1 s: consistent with wall contact (inferred from pose and map; contact sensors not recorded). Not stopping performance. On the robot the MCU is the last hop;
  loss of the link to the MCU was not tested.
- N4: SIGKILL + instant respawn of the bridge keeps the writer guard down 20.0-20.5 s (DDS lease), stand-in RUN again at 20.8 s; fail-safe, no mitigation in rclpy.
- Overnight soak (44 sessions, kit 8d7211e): 40 valid; the 4 invalid are session 11 of each instance, aborted by the SIGTERM that stopped the soak ("failed checks" are consequences). Two
  default-profile soft stops (instance 2, sessions 9 and 11), each 1-2 s into the first goal of the session, host load average ~19 on 12 threads: aggregator stale drive_supervision
  (+odom), 400 ms heartbeat gap, stand-in back in RUN in 0.35 s. Cause (starved watcher process) not determined.
- Not measured: other jobs, AMCL, hardware, GPU, BT-triggered recoveries, runs over 1 h per session on the fixed code.
