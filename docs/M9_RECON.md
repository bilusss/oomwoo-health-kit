# M9: in-node work-path heartbeats (dock_detector, dock_drive, coverage_planner) on the full sim stack

Dates 2026-11-18 (recon, dock fault trials) and 2026-11-20/21 (coverage fault trials, soaks). Maintainer's answer to design question (2026-11-18): static roster with
`drive_supervision`, topic watchers only as the fallback for third-party nodes, OOMWOO's own nodes send in-node work-path heartbeats, the cmd_vel gate sits at the MCU
bridge. Firmware PR #3 (makerspet/oomwoo-firmware): a heartbeat loss makes the motor outputs safe but keeps the shared rail on, so the LiDAR stays up; only the RC
watchdog and hard faults cut the rail. The kit's MCU stand-in already behaves this way; no change.

## 1. What was built

Upstream patch (local only, nothing pushed): `~/projects/oomwoo-ros2-tools`, branch `health-in-node-heartbeats`, base 338e437, commits 44717b0 (package), 689e057 (dock),
40552ff (coverage).

- New ament_python package `oomwoo_health_beat`: `core.py` (payload builder, no ROS imports; fields `component_id`, `health`, `stamp_sec`, `sequence`, `detail`) and
  `ros.py` (`HealthBeat`: declares the node parameter `health_component_id`, default `""` = `beat()` returns at once and no publisher is created). Topic
  `/oomwoo/health/component`, `std_msgs/String`, reliable, volatile, depth 10, stamp = the node's ROS clock. `README.md` lists the beat point of each node and why.
- `dock_detector`: beat after each scan that passed the `detect_hz` gate and was processed (`_on_scan` -> `_process_scan`), found or not. `dock_drive`: beat after each 20 Hz
  control tick that computed and published its command (`_tick` -> `_step`), in every state including IDLE. `coverage_planner`: beat after each 5 Hz `_tick` -> `_step`, which
  holds everything that advances the executor (dispatch, watchdog, wedge escape, reactive drive); same code for `executor:=nav2` and `reactive`. All beats are level `ok`.
- Build/test in the dev image as an overlay on `/ros_ws`: `colcon test` oomwoo_health_beat 16 (10 core, 3 ROS wrapper incl. "disabled by default creates no publisher",
  3 linters), oomwoo_dock 15, oomwoo_coverage 21: 52 tests, 0 failures (flake8, pep257, copyright pass).

Limits of what a beat proves: the loop of that node runs and is not blocked. A planner without a pose or with all waypoints skipped still beats (its own watchdogs handle
that); a coverage planner that finished ("coverage complete") and idles beats too. A detector that is alive but fits the wrong thing beats.

Kit side: `tools/sim.sh overlay-build` / `OHK_OVERLAY=1 tools/sim.sh up N` (patched packages shadow the image's; commit recorded as `tools_commit` in every record),
tasks `dock_inode`, `coverage_inode`, `coverage_reactive_inode` (+ `*_recon` with 5 s max_age), `bench/fullstack/beat_trace.py` + `beat_stats.py` (recon), `beat_relay.py`
(section 3), `collect_m9.py`, `tools/m9_trace.sh`, `tools/m9_chain.sh`. The coverage planner is started by the worker BEFORE the loop arms for these tasks
(`CoverageJob.prestart`, once Nav2 is active and the robot localized), because a critical planner component keeps the roster from arming otherwise; the stand-in gates
cmd_vel until armed.

## 2. Recon: beat periods and max_age_sec

Method: one 1800 sim s session per task with the recon roster (5 s max_age, so the loop does not react) and `beat_trace.py` on `/oomwoo/health/component_node` (the
nodes' own topic, stamps = sim clock of the node), cut where the next stack restarts the clock (`beat_stats.load`). Rule for `max_age_sec` written down BEFORE the data was
read (`beat_stats.py` docstring, test `test_age_before_refresh_and_recommended_max_age`): `max(3 x median period, 1.5 x the largest "age before refresh")`, rounded up to
0.05 s, where age before refresh = sim time at the receipt of the next beat minus the stamp of the previous one (gap plus delivery delay).

| Component | Session(s) | Beats | Period p50 / p95 / p99 / max (s, by stamp) | Largest age before refresh (s) | moving / stationary max period | Seq gaps | max_age_sec set |
|---|---|---|---|---|---|---|---|
| dock_detector | dock, GPU, 1801 sim s | 3006 intervals | 0.600 / 0.602 / 0.604 / 0.614 | 0.762 | 0.614 / 0.612 | 0 | **1.8** |
| dock_drive | same | 36091 | 0.050 / 0.050 / 0.050 / 0.061 | 0.061 | 0.053 / 0.061 | 0 | **0.15** |
| coverage_planner, nav2 | 2 sessions (CPU and GPU), 1800 sim s each | 18013 | 0.200 / 0.200 / 0.200 / 0.202 | 0.210 | 0.201 / 0.202 | 0 | **0.6** |
| coverage_planner, reactive | CPU, 1800 sim s (valid, RTF 0.988) | 9006 | 0.200 / 0.200 / 0.201 / 0.204 | 0.400 | 0.203 / 0.204 | 0 | **0.6** |

Commands: `python3 bench/fullstack/beat_stats.py results/fullstack/m9/1/beat_trace_dock_gpu.jsonl` (also `--json`), `.../2/beat_trace_coverage_reactive_cpu_clean.jsonl`,
`.../1/beat_trace_coverage_nav2_cpu.jsonl .../2/beat_trace_coverage_nav2_gpu.jsonl`. Per-segment rows (moving, stationary, dock_drive state) are printed by the same command.

- The idle case is covered by the stationary segments: coverage sessions finish the sweep in about 600-1000 s and then idle with the planner beating (nav2: 8871 of the
  18013 intervals are stationary), the dock detector and driver run between attempts (dock_drive state IDLE: 4610 intervals, max 0.053 s; detector in IDLE max 0.607 s).
- The ticks do not slow down while driving: the "up to 0.60 s" seen in M8 for the reactive executor was the COMMAND stream (/cmd_vel), not the planner tick. The tick itself
  never exceeded 0.204 s in 9006 intervals. The largest age before refresh in the reactive session (0.400 s) is a delivery delay of a stationary beat (age at receipt 0.196 s),
  not a slow tick.
- The 3 x period floor decided every value (1.8, 0.15, 0.6); the 1.5 x largest-age term was smaller in all four rows.
- Failed and invalid recon sessions, kept in `results/fullstack/m9/2/sessions.jsonl` (`published/recon_sessions.jsonl`): reactive recon crashed twice 19 s after the start (gz
  SIGSEGV, GPU container that had lost GPU access, section 7); a third reactive recon (`m9_recon_retry2`) was INVALID on RTF (0.899) because it ran while another instance
  restarted stacks every minute for fault trials (planner tick max 0.449 s there, 4 sequence gaps from the later trials, not used); the clean repeat is `m9_recon_retry3`.
- GPU vs CPU (the requested comparison): the same task (coverage, nav2 executor, 1800 sim s) started at the same time in two containers: 4.093 cores container CPU without
  `--gpu`, 4.064 with it (RTF 0.9997 in both), i.e. 0.029 cores (0.7%) cheaper with the GPU, within run-to-run noise. The rule "use the cheaper one" selected the GPU, but
  containers lost GPU access after about 30 minutes (`nvidia-smi` in the container: "Failed to initialize NVML: Unknown Error"), after which stack starts crash, so everything
  from 2026-11-18 22:49 on (36 of the 45 dock fault trials counted below, all coverage trials, all soaks) used CPU rendering; the first 9 sigkill_dock_detector trials and the dock recon
  used the GPU (`gpu` field of each record).

## 3. Why the sim-clock-to-wall-clock relay (`bench/fullstack/beat_relay.py`, commit dab4e14) exists, and why it cannot hide a stale beat

Needed because: the nodes stamp with their ROS clock, which in Gazebo is `/clock` (sim time) received separately by every process. The aggregator (b2b9796) treats a heartbeat
newer than its own clock (age < 0) as stale. A beat stamped on the very `/clock` tick the sender just received and published at once (dock_drive, 20 Hz) reaches an aggregator
whose own `/clock` is one tick behind in a small fraction of its 20 Hz evaluations; one such evaluation is a fault episode of about 0.3 s (arming window 0.25 s) = a soft
stop. Measured with everything on sim time (`results/fullstack/m9/_smoke1`, 240 sim s, 5 s max_age): **28 soft stops, 20 blamed on dock_drive**; with the relay
(`_smoke2`, 301 sim s): 0. It is a simulation artefact: on the robot all processes share the system clock and a message cannot arrive before it was stamped.

What it does: the instrumented nodes are launched with `/oomwoo/health/component` remapped to `/oomwoo/health/component_node`. The relay (sim time) forwards each beat's JSON
unchanged except `stamp_sec_out = wall_now - max(0, sim_now - stamp_sec)` (the beat's age in sim seconds moved onto the wall clock the aggregator and watcher use, as in M7/M8).

Why it cannot hide a stale beat:
1. It has no timer, no stored beat and no output without an input: `BeatRelay.on_beat` returns at most one message per received message (`test_relay_cannot_make_a_beat_fresher_or_invent_one`). A
   node that stops (SIGKILL, SIGSTOP, wedged loop) stops producing relayed beats and the aggregator's age of its last beat only grows (same test, last block).
2. The age the aggregator can compute from a relayed beat is never smaller than the age at receipt in sim seconds (`wall_now - stamp_out >= sim_now - stamp_in`, same test); the
   clamp at 0 only removes negative ages, i.e. the artefact. Order of beats and `sequence` are preserved.
3. The measured evidence is the fault trials below: all 45 dock_detector/dock_drive and 30 coverage_planner kills and freezes, whose only difference to a healthy run is that
   the node's beats stop, were detected and blamed on the in-node component through the relay.
A dead relay makes every in-node component stale (fail-safe, and it is a key process of the session).

Could the relay be dropped? Not without changing either the unit under test or the aggregator: (a) run the nodes on wall time: possible for the two dock nodes but not for the
planner (its Nav2 goals are stamped with the node clock and must be in sim time for the TF lookups); (b) a tolerance for small negative ages in the aggregator changes the
component whose behaviour is being verified; (c) putting every health process on sim time still races (the 28 stops above). The relay is the least invasive option; the
real patch to the nodes stays contract-pure (stamp from the node's ROS clock).

## 4. Fault trials (same method as M7/M8 per-node faults: kill and SIGSTOP freeze 5 s, one stack per trial)

Triggers as in M7/M8: dock faults while `dock_drive` is in BACK and the robot moves >= 0.03 m/s for 2 s; coverage nav2 while moving >= 0.12 m/s (Nav2 transit at 0.2 m/s);
coverage reactive while >= 0.30 m/s (the planner's own pass). All trials counted: valid, exposed, 0 not stopped at the end. Raw: `results/fullstack/m9/1/faults.jsonl` (dock),
`m9/1/faults.jsonl` (coverage nav2) and `m9/2/faults.jsonl` (coverage reactive); one trial was aborted by my SIGTERM and is not counted (invalid, kept in the raw file). Table
`python3 bench/fullstack/summarize_faults.py results/fullstack/m9/published/faults_m9.jsonl --label=m9_faults`; all `config_sha256` and the commits are in each record.

| Fault | Roster | Detected / exposed | Zero at the gate s (min / med / max) | Distance to standstill m | Rotation after the fault, deg | Blamed | Baseline (M7 -> M8) |
|---|---|---|---|---|---|---|---|
| SIGKILL dock_detector (reversing) | dock_detector 1.8 s | **20 / 20** | 1.26 / 1.55 / 1.88 | 0.07 / 0.09 / 0.11 | 0.0 / 0.2 / 0.7 | dock_detector | M7 0/5 (zero 2.61-3.00, 0.16-0.18 m); M8 rule 10/10 (1.42-2.00, 0.09-0.12 m) |
| SIGSTOP dock_detector 5 s | same | **10 / 10** (10 resumed) | 1.25 / 1.62 / 1.76 | 0.07 / 0.09 / 0.10 | 0.1 / 0.3 / 0.6 | dock_detector | M7 0/5; M8 10/10 (1.40-1.99) |
| SIGKILL dock_drive (reversing) | dock_drive 0.15 s | 10 / 10 | 0.23 / 0.25 / 0.30 | 0.01 / 0.01 / 0.02 | 0.0 / 0.0 / 0.2 | dock_drive + drive_supervision | M7 5/5 (0.35-0.41, 0.02-0.03 m) |
| SIGSTOP dock_drive 5 s | same | 5 / 5 (5 resumed) | 0.24 / 0.27 / 0.28 | 0.01 / 0.02 / 0.02 | 0.0 / 0.1 / 0.3 | dock_drive + drive_supervision | M7 5/5 (0.32-0.41) |
| SIGKILL coverage_planner, nav2 executor (Nav2 transit) | coverage_planner 0.6 s | **10 / 10** | 0.54 / 0.62 / 0.72 | 0.12 / 0.13 / 0.15 | 4.1 / 5.5 / 6.6 | coverage_planner | M7 0/5 (zero 6.0-6.2 s, 0.49-0.51 m); not closed in M8 |
| SIGSTOP coverage_planner 5 s, nav2 | same | **5 / 5** (5 resumed) | 0.53 / 0.61 / 0.71 | 0.11 / 0.13 / 0.15 | 4.1 / 6.0 / 6.2 | coverage_planner | M7 0/5 (robot still moving at the end 5/5) |
| SIGKILL coverage_planner, reactive (>= 0.30 m/s) | same value, watch config nav_watch_amcl_reactive.yaml | 10 / 10 | 0.39 / 0.43 / 0.57 | 0.16 / 0.18 / 0.23 | 0.3 / 2.7 / 8.2 | coverage_planner + drive_supervision | M7 5/5 (0.42-0.55, 0.18-0.22 m) via drive_supervision |
| SIGSTOP coverage_planner 5 s, reactive | same | 5 / 5 (5 resumed) | 0.39 / 0.41 / 0.58 | 0.16 / 0.17 / 0.23 | 2.1 / 5.1 / 8.3 | coverage_planner + drive_supervision | M7 5/5 (0.37-0.54) |

Time from the fault to the first aggregator fault (min/med/max, s): dock_detector kill 1.16 / 1.45 / 1.77, freeze 1.15 / 1.52 / 1.65; planner nav2 kill 0.43 / 0.51 / 0.62; reactive
0.28 / 0.33 / 0.46; dock_drive 0.12 / 0.14 / 0.19.

Reading the table:
- The two rows that were blind in M7 are closed: dock_detector 30/30 (kill + freeze), nav2-executor coverage_planner 15/15. Stop times follow the roster values (the planner at 0.54-0.72 s = max_age 0.6 s plus
  up to 0.2 s since the last beat plus aggregator and stand-in; the detector 1.26-1.88 s with max_age 1.8 s, since the beat can be up to 0.6 s old when the fault hits).
- Against M8's `reverse_fix` the dock_detector result is the same within the spread (zero 1.26-1.88 vs 1.42-2.11 s; reversing 0.07-0.11 vs 0.09-0.13 m): the in-node beat does not stop faster, because
  the 1.8 s bound follows from the 0.6 s fit period. What differs is the kind of guarantee (see the soak below: no stops from the detector beat itself, no precondition about other writers).
- The nav2 planner kill now stops the robot in 0.13 m instead of letting it drive on for 0.5 m (kill) or indefinitely (freeze). This is a designed consequence: a dead job planner now stops the
  robot (the stop reason is "coverage_planner stale", not a motion fault).
- Reactive and dock_drive rows were already covered by `drive_supervision`; the in-node beat adds the correct blame and is not slower (reactive 0.39-0.57 vs M7 0.42-0.55 s). The faster dock_drive times (0.23-0.30 vs
  0.35-0.41 s) are observed, the mechanism was not isolated (load and n differ).
- Rotation after the fault is below 9 degrees in every trial (the M7/M8 metric).

## 5. False-trip soak (6.5 sim hours, in-node rosters, CPU rendering, two instances at a time)

Sessions of 1800 sim s, all valid (RTF 0.9887-0.9994, worst 30 s window 0.981, armed, key processes alive, `beat_relay` among them), label `m9_soak`; chains in `results/fullstack/m9/_chains/soak_inst{1,2}.txt` plus the first
nav2-coverage session (started together with the clean reactive recon). Table: `python3 bench/fullstack/summarize.py --by-task results/fullstack/m9/published/soak_m9_*.jsonl`.

| Task | Roster | Sessions | Sim h (= active h) | Soft stops | Stops blamed on an in-node component | Stops not explained by an injected fault or harness action | Per active hour |
|---|---|---|---|---|---|---|---|
| dock (6 start poses, harness teleport between attempts) | M7 dock + dock_detector 1.8 + dock_drive 0.15 | 5 | 2.5 | 33 | **0** | **0** (all 33 within 0.5 s after a harness `set_pose`, blamed `drive_supervision`; 271 attempts docked, 9 timed out, 5 cut by the session end) | 13.2 (all harness-induced) |
| coverage, nav2 executor | M7 AMCL + coverage_planner 0.6 | 3 | 1.5 | **0** | 0 | 0 | 0 (bound about 2.0/h at 95%, 3/T) |
| coverage, reactive executor (proposal watch config, cmd 0.30 s) | same | 3 | 1.5 | 32 | **0** | 32, all blamed on `drive_supervision` (the known reactive-command mismatch: M7 recorded 8 and 11 per 0.5 h on this config) | 21.3 |
| AMCL navigation, mix workload | M7 AMCL roster (no in-node component exists on this stack) | 2 | 1.0 | **0** | 0 | 0 | 0 (bound 3.0/h) |
| Total | | 13 | 6.5 | 65 | **0** | 32 (reactive, drive_supervision) | |

- The three in-node components were never blamed for a soft stop in 6.5 h; each also beat through the stationary/idle phases. This includes the dock detector, where a 1.8 s window is above the 0.76 s largest age.
- The reactive-executor stops are not an M9 effect: same blame, phase ("en route") and rate as the M7 proposal-config rows (8 and 11 stops in 0.5 h sessions = 16 and 22 per active hour; here 8, 12, 12 per session = 16-24 per hour). They stay open from M7/M8 and are not solved by the in-node beat.
- Soft stops "explained" by harness actions: the dock stops follow the teleport (`set_pose`) between attempts as in M7; this classification is the one `summarize.py` has used since M7 (0.0-0.5 s window).
- Host load (1-min average sampled every 10 s, median of session medians, max): dock 15.5 / 22.0, coverage nav2 15.6 / 29.9, reactive 15.7 / 22.9, AMCL nav 3.7 / 15.3 of 12 logical cores (two instances at a time; M7: 7-10 busy cores).
  The AMCL nav sessions ran with the other instance idle.
- Container CPU per session (`cpu_cores_container`): dock 3.02-3.04, coverage nav2 3.72-3.74, reactive 3.68-3.71, AMCL navigation 2.19-2.21 cores.

## 6. Commands as executed

    # overlay and containers (CPU; every call with OHK_RES_SUB=m9 OHK_OVERLAY=1)
    tools/sim.sh overlay-build ; tools/sim.sh up 2
    # recon (one session per task) and traces
    tools/sim.sh run 1 --health-sources watcher --health-clock wall --task coverage_inode_recon --duration-sim 1800 --sessions 1 --profile default --label m9_recon
    tools/m9_trace.sh 1 2200 beat_trace_coverage_nav2_cpu            # likewise dock_inode_recon (GPU, label m9_recon) and coverage_reactive_inode_recon (label m9_recon_retry3)
    # fault trials: chain files results/fullstack/m9/_chains/faults_inst1{,_cpu}.txt faults_cov_inst{1,2}.txt, run with tools/m9_chain.sh <i> <file>
    # soaks: results/fullstack/m9/_chains/soak_inst{1,2}.txt
    python3 bench/fullstack/collect_m9.py ; python3 bench/fullstack/summarize_faults.py results/fullstack/m9/published/faults_m9.jsonl --label=m9_faults
    python3 bench/fullstack/summarize.py --by-task results/fullstack/m9/published/soak_m9_*.jsonl
    # tests
    uv run --no-project --with pytest --with pyyaml pytest -q       # host
    tools/dev.sh build && tools/dev.sh test                          # container (run when no sim is running)

## 7. Limits and things that went wrong

- Aggregator pinned at b2b9796; firmware not involved (the stand-in is the MCU model). Sim only: no real-time clock effects, no real load on the beat path.
- The relay (section 3) and the pre-arm start of the planner are harness choices, both documented; the first nine dock_detector kill trials and the dock recon were rendered on the GPU, everything else on CPU.
- Dock detector kills: 9 were taken on the GPU before it was abandoned, 11 on CPU (plus the freezes on CPU); not separated in the table, the whole series lies in 1.26-1.88 s.
- Fault trials ran with the other instance also running trials (stack starts of one instance overlap the other's trials; host load median 7-36 in the table of section 4); M7/M8 had the same arrangement.
- Aborted by my SIGTERM: one dock kill trial (invalid, not counted). Invalid recon sessions listed in section 2.
- The reactive-executor false-stop rate (21.3 per active hour) is unchanged from M7 and unrelated to the beats; the nav2-executor soak has 1.5 h and bounds the false-stop rate only to about 2/h at 95%.
- Beat placement in the planner counts waiting/idle ticks as work; a planner that is alive but cannot advance (no pose, no map) is not detected by the beat.
- Not done: in-node beats for Nav2 and slam_toolbox (third-party, watcher fallback stays), real-hardware timing.
