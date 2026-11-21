# M9: in-node work-path heartbeats (dock_detector, dock_drive, coverage_planner) on the full sim stack

Dates 2026-11-18 (recon, dock fault trials) and 2026-11-20/21 (coverage fault trials, soaks). Maintainer's answer to design question (2026-11-18): static roster with
`drive_supervision`, topic watchers only as the fallback for third-party nodes, OOMWOO's own nodes send in-node work-path heartbeats, the cmd_vel gate sits at the MCU
bridge. Firmware PR #3 (makerspet/oomwoo-firmware): a heartbeat loss makes the motor outputs safe but keeps the shared rail on, so the LiDAR stays up; only the RC
watchdog and hard faults cut the rail. The kit's MCU stand-in already behaves this way; no change.

## 1. What was built

Upstream patch (local only, nothing pushed): `~/projects/oomwoo-ros2-tools`, branch `health-in-node-heartbeats`, base 338e437, commits 44717b0 (package), 689e057 (dock),
40552ff (coverage); all M9 data were taken at 40552ff. Review follow-up cb088fd: `stamp_sec` is now `nanoseconds / 1e9` instead of `nanoseconds * 1e-9`
(section 3; the value differs by at most 1 ulp, and the measurements below went through the relay, so they are unaffected).

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

- The idle case is covered by the stationary segments: coverage sessions finish the sweep (nav2 executor in about 950-1020 s, reactive in about 380-560 s) and then idle
  with the planner beating (nav2: 8871 of the 18013 intervals are stationary), the dock detector and driver run between attempts (dock_drive state IDLE: 3953 intervals,
  max 0.053 s; detector in IDLE max 0.607 s).
- The ticks do not slow down while driving: the "up to 0.60 s" seen in M8 for the reactive executor was the COMMAND stream (/cmd_vel), not the planner tick. The tick itself
  never exceeded 0.204 s in 9006 intervals. The largest age before refresh in the reactive session (0.400 s) is a delivery delay of a stationary beat (age at receipt 0.196 s),
  not a slow tick.
- The 3 x period floor decided every value (1.8, 0.15, 0.6); the 1.5 x largest-age term was smaller in three rows and equal in the reactive row (1.5 x 0.400 = 0.60).
  For dock_detector the age term alone would give 1.15 s: 1.8 s is three fit periods by the rule, not a value the data required, and it makes the detector the slowest
  stop in the table (section 4).
- Failed and invalid recon sessions, kept in `results/fullstack/m9/2/sessions.jsonl` (`published/recon_sessions.jsonl`): reactive recon crashed twice 19 s after the start (gz
  SIGSEGV, GPU container that had lost GPU access, section 7); a third reactive recon (`m9_recon_retry2`) was INVALID on RTF (0.899) because it ran while another instance
  restarted stacks every minute for fault trials (planner tick max 0.449 s there, 0 sequence gaps; not used, but the rule applied to that loaded trace would give 0.70 s instead of 0.6); the clean repeat is `m9_recon_retry3`.
- GPU vs CPU (the requested comparison): the same task (coverage, nav2 executor, 1800 sim s) started at the same time in two containers: 4.093 cores container CPU without
  `--gpu`, 4.064 with it (RTF 0.9997 in both), i.e. 0.029 cores (0.7%) cheaper with the GPU, within run-to-run noise. The rule "use the cheaper one" selected the GPU, but
  containers lost GPU access after about 30 minutes (`nvidia-smi` in the container: "Failed to initialize NVML: Unknown Error"), after which stack starts crash, so everything
  from 2026-11-18 22:49 on (36 of the 45 dock fault trials counted below, all coverage trials, all soaks) used CPU rendering; the first 9 sigkill_dock_detector trials and the dock recon
  used the GPU (`gpu` field of each record).

## 3. Why the sim-clock-to-wall-clock relay (`bench/fullstack/beat_relay.py`, commit dab4e14) exists, and why it cannot hide a stale beat

Built because: with everything on sim time (`results/fullstack/m9/_smoke1`, 240 sim s, 5 s max_age) there were **28 soft stops, 27 blamed on dock_drive alone** (1 on
`drive_supervision` + `odom`); with the relay (`_smoke2`, 301 sim s): 0. The aggregator (b2b9796, `oomwoo_health_monitor/core.py` line 131,
`if age_sec < 0.0 or age_sec > spec.max_age_sec`) counts a beat newer than its own clock as stale; one such 20 Hz evaluation is a fault episode of about 0.3 s (arming
window 0.25 s) = a soft stop.

Mechanism (corrected in the review of 2026-11-21; the first version blamed a `/clock` tick reaching the aggregator late): float rounding on an EQUAL tick. The aggregator
reads its clock as `nanoseconds / 1_000_000_000.0` (`health_monitor_node.py` line 82); the patch at 40552ff stamped `nanoseconds * 1e-9`, which for the same nanosecond is
1 ulp larger for about 41% of values (150 ms: 0.15000000000000002 vs 0.15). dock_drive's 20 Hz timer and the aggregator's 20 Hz timer fire on the same `/clock` ticks
(dock_drive stamps are exact multiples of 50 ms in 2372 of the 2374 smoke1 beats); when the aggregator takes the beat before its own timer callback of that tick, the age is
0 exactly and -1 ulp in floats, i.e. stale. Evidence: (1) the beat tracer, a third sim-time process, received 0 of 2374 (smoke1) and 0 of 3990 (smoke2) dock_drive beats
stamped ahead of its own clock, and 2077 of the 2374 on exactly its own tick; (2) the relay uses the same `* 1e-9` as the nodes and logged `clamped=0` in all 22 sessions long enough to
log its 60 s counter (330 674 beats, never one ahead of its clock); (3) targeted check (`results/fullstack/m9/_review_clock/`: aggregator b2b9796 on a 1 kHz sim clock, two producers on 20 Hz
sim timers in separate processes, 5 s max_age): 120 s unloaded, `* 1e-9` stale in 3 evaluations, `/ 1e9` in 0; 180 s with the host loaded (9 busy loops), 232 (217 fault
onsets) vs 0. Fixed in the patch by cb088fd (`stamp_sec = nanoseconds / 1e9`). On the robot the aggregator reads its clock after the beat was stamped, so equal nanoseconds
and this artefact do not occur.

What it does: the instrumented nodes are launched with `/oomwoo/health/component` remapped to `/oomwoo/health/component_node`. The relay (sim time) forwards each beat's JSON
unchanged except `stamp_sec_out = wall_now - max(0, sim_now - stamp_sec)` (the beat's age in sim seconds moved onto the wall clock the aggregator and watcher use, as in M7/M8).

Why it cannot hide a stale beat:
1. It has no timer, no stored beat and no output without an input: `BeatRelay.on_beat` returns at most one message per received message (`test_relay_cannot_make_a_beat_fresher_or_invent_one`). A
   node that stops (SIGKILL, SIGSTOP, wedged loop) stops producing relayed beats and the aggregator's age of its last beat only grows (same test, last block).
2. The age the aggregator can compute from a relayed beat is never smaller than the age at receipt in sim seconds (`wall_now - stamp_out >= sim_now - stamp_in`, same test); the
   clamp at 0 only removes negative ages. Order of beats and `sequence` are preserved. Limit: `sim_now` is the relay's own `/clock`; had the relay's clock lagged the nodes',
   the clamp would have hidden that lag (age 0 instead of the true age). `clamped=0` in every logged counter shows this never happened. If `/clock` stalls for everyone, the
   beats stop (all three beat points run on sim-time timers or, for the detector, a sim-time rate gate), nothing is forwarded and the beats go stale on the wall clock.
   The unit test covers the conversion and the clamp; its "node stopped" block only checks arithmetic on outputs already produced (the property is structural: no timer,
   no stored beat), and no test covers a lagging relay clock.
3. The measured evidence is the fault trials below: all 45 dock_detector/dock_drive and 30 coverage_planner kills and freezes, whose only difference to a healthy run is that
   the node's beats stop, were detected and blamed on the in-node component through the relay.
A dead relay makes every in-node component stale (fail-safe, and it is a key process of the session).

Could the relay be dropped? Not without changing either the unit under test or the aggregator: (a) run the nodes on wall time: possible for the two dock nodes but not for the
planner (its Nav2 goals are stamped with the node clock and must be in sim time for the TF lookups); (b) a tolerance for small negative ages in the aggregator changes the
component whose behaviour is being verified; (c) putting every health process on sim time: with the 40552ff stamp it tripped (the 28 stops above); with the division stamp
the targeted check had 0 stale evaluations (300 s emulated, plus 120 s loaded with cb088fd itself), but the full stack was not re-run that way. The relay is what the M9 numbers were measured with; the patch to the
nodes stays contract-pure (stamp from the node's ROS clock).

## 4. Fault trials (same method as M7/M8 per-node faults: kill and SIGSTOP freeze 5 s, one stack per trial)

Triggers as in M7/M8: dock faults while `dock_drive` is in BACK and the robot moves >= 0.03 m/s for 2 s; coverage nav2 while moving >= 0.12 m/s (Nav2 transit at 0.2 m/s);
coverage reactive while >= 0.30 m/s (the planner's own pass). All trials counted: valid, exposed, 0 not stopped at the end. Raw: `results/fullstack/m9/1/faults.jsonl` (dock),
`m9/1/faults.jsonl` (coverage nav2) and `m9/2/faults.jsonl` (coverage reactive); one trial was aborted by my SIGTERM and is not counted (invalid, kept in the raw file). Table
`python3 bench/fullstack/summarize_faults.py results/fullstack/m9/published/faults_m9.jsonl --label=m9_faults`; all `config_sha256` and the commits are in each record.

| Fault | Roster | Detected / exposed | Zero at the gate s (min / med / max) | Distance to standstill m | Rotation after the fault, deg | Blamed | Baseline (M7 -> M8) |
|---|---|---|---|---|---|---|---|
| SIGKILL dock_detector (reversing) | dock_detector 1.8 s | **20 / 20** | 1.26 / 1.55 / 1.88 | 0.07 / 0.09 / 0.11 | 0.0 / 0.2 / 0.7 | dock_detector | M7 0/5 (zero 2.61-3.00, 0.16-0.18 m); M8 rule 10/10 (zero 1.52-2.11, 0.09-0.12 m) |
| SIGSTOP dock_detector 5 s | same | **10 / 10** (10 resumed) | 1.25 / 1.62 / 1.76 | 0.07 / 0.09 / 0.10 | 0.1 / 0.3 / 0.6 | dock_detector | M7 0/5; M8 10/10 (zero 1.50-2.10) |
| SIGKILL dock_drive (reversing) | dock_drive 0.15 s | 10 / 10 | 0.23 / 0.25 / 0.30 | 0.01 / 0.01 / 0.02 | 0.0 / 0.0 / 0.2 | dock_drive first (10/10), drive_supervision 0.10-0.15 s later | M7 5/5 (0.35-0.41, 0.02-0.03 m) |
| SIGSTOP dock_drive 5 s | same | 5 / 5 (5 resumed) | 0.24 / 0.27 / 0.28 | 0.01 / 0.02 / 0.02 | 0.0 / 0.1 / 0.3 | dock_drive first (5/5), drive_supervision 0.10-0.15 s later | M7 5/5 (0.32-0.41) |
| SIGKILL coverage_planner, nav2 executor (Nav2 transit) | coverage_planner 0.6 s | **10 / 10** | 0.54 / 0.62 / 0.72 | 0.12 / 0.13 / 0.15 | 4.1 / 5.5 / 6.6 | coverage_planner | M7 0/5 (zero 6.0-6.2 s, one 11.2 s; 0.49-0.51 m); not closed in M8 |
| SIGSTOP coverage_planner 5 s, nav2 | same | **5 / 5** (5 resumed) | 0.53 / 0.61 / 0.71 | 0.11 / 0.13 / 0.15 | 4.1 / 6.0 / 6.2 | coverage_planner | M7 0/5 (robot still moving at the end 5/5) |
| SIGKILL coverage_planner, reactive (>= 0.30 m/s) | same value, watch config nav_watch_amcl_reactive.yaml | 10 / 10 | 0.39 / 0.43 / 0.57 | 0.16 / 0.18 / 0.23 | 0.3 / 2.7 / 8.2 | drive_supervision first (10/10), coverage_planner 0.15-0.20 s later | M7 5/5 (0.42-0.55, 0.18-0.22 m) via drive_supervision |
| SIGSTOP coverage_planner 5 s, reactive | same | 5 / 5 (5 resumed) | 0.39 / 0.41 / 0.58 | 0.16 / 0.17 / 0.23 | 2.1 / 5.1 / 8.3 | drive_supervision first (5/5), coverage_planner 0.15-0.20 s later | M7 5/5 (0.37-0.54) |

Time from the fault to the first aggregator fault (min/med/max, s): dock_detector kill 1.16 / 1.45 / 1.77, freeze 1.15 / 1.52 / 1.65; planner nav2 kill 0.43 / 0.51 / 0.62; reactive
kill 0.28 / 0.33 / 0.46 (this is `drive_supervision`; `coverage_planner` was first named at 0.43 / 0.48 / 0.63, kill and freeze, always after the stand-in had already
stopped); dock_drive kill + freeze 0.12 / 0.15 / 0.19 (`dock_drive`; `drive_supervision` followed at 0.23-0.32).

Reading the table:
- The two rows that were blind in M7 are closed: dock_detector 30/30 (kill + freeze), nav2-executor coverage_planner 15/15. Stop times follow the roster values: max_age minus the age of the last beat when the
  fault hits, plus up to one aggregator period and the stand-in's 0.15 s (planner 0.53-0.72 s with max_age 0.6 s and beats 0.2 s apart; detector 1.25-1.88 s with max_age
  1.8 s and beats 0.6 s apart).
- Against M8's `reverse_fix` the dock_detector result is the same within the spread (zero at the gate 1.25-1.88 vs 1.50-2.11 s; reversing 0.07-0.11 vs 0.09-0.13 m): the in-node beat does not stop faster, because
  the 1.8 s bound follows from the 0.6 s fit period. What differs is the kind of guarantee (see the soak below: no stops from the detector beat itself, no precondition about other writers).
- The nav2 planner kill now stops the robot in 0.13 m instead of letting it drive on for 0.5 m (kill) or indefinitely (freeze). This is a designed consequence: a dead job planner now stops the
  robot (the stop reason is "coverage_planner stale", not a motion fault).
- Reactive and dock_drive rows were already covered by `drive_supervision`. dock_drive: the in-node beat (0.15 s, 20 Hz) was the first fault in 15/15 trials, 0.10-0.15 s
  before `drive_supervision`, and that is where the earlier stop comes from (0.23-0.30 vs M7 0.35-0.41 s). Reactive: `drive_supervision` was first in 15/15 and stopped the
  robot; `coverage_planner` was named 0.15-0.20 s later, after the stop in every trial. So the reactive stop times (0.39-0.58 vs M7 0.37-0.55 s) are `drive_supervision`'s and
  say nothing about the in-node beat's speed; on this executor the beat adds the blame, not the stop.
- Rotation after the fault is below 9 degrees in every trial (the M7/M8 metric).

## 5. False-trip soak (6.5 sim hours, in-node rosters, CPU rendering, two instances at a time)

Sessions of 1800 sim s, all valid (RTF 0.9887-0.9994, worst 30 s window 0.981, armed, key processes alive, `beat_relay` among them in the in-node tasks), label `m9_soak`; chains in `results/fullstack/m9/_chains/soak_inst{1,2}.txt` plus the first
nav2-coverage session (started together with the clean reactive recon). Table: `python3 bench/fullstack/summarize.py --by-task results/fullstack/m9/published/soak_m9_*.jsonl`.

| Task | Roster | Sessions | Sim h | Active h | Soft stops | Stops blamed on an in-node component | Stops not explained by an injected fault or harness action | Per active hour |
|---|---|---|---|---|---|---|---|---|
| dock (6 start poses, harness teleport between attempts) | M7 dock + dock_detector 1.8 + dock_drive 0.15 | 5 | 2.5 | 2.5 | 33 | **0** | **0** (all 33 came 0.22-0.26 s after a harness `set_pose`, blamed `drive_supervision`; 271 attempts docked, 9 timed out, 5 cut by the session end) | 13.2 (all harness-induced) |
| coverage, nav2 executor | M7 AMCL + coverage_planner 0.6 | 3 | 1.5 | 0.81 | **0** | 0 | 0 | 0 (bound about 3.7/h at 95%, 3/T) |
| coverage, reactive executor (proposal watch config, cmd 0.30 s) | same | 3 | 1.5 | 0.35 | 32 | **0** | 32, all blamed on `drive_supervision` (the known reactive-command mismatch: M7 recorded 8 in 0.14 and 11 in 0.12 active h on this config) | 90 |
| AMCL navigation, mix workload | M7 AMCL roster (no in-node component exists on this stack) | 2 | 1.0 | 1.0 | **0** | 0 | 0 | 0 (bound 3.0/h) |
| Total | | 13 | 6.5 | 4.67 | 65 | **0** | 32 (reactive, drive_supervision) | |

Active time as in M7 (`summarize.py`, `active_seconds`): coverage jobs finish the sweep and idle, so their active time ends at the last motion evidence in the record
(nav2 953-989 s per session; reactive 378-483 s, where the last evidence the record keeps is the last soft stop, so 0.35 h may be short: with the clean recon's ~555 s
sweep the reactive rate would be about 70 per active hour). The first version of this table counted the whole window as active (1.5 h per coverage row, 21.3/h and a
2.0/h bound), because `summarize.py` did not list the M9 coverage task names; fixed in commit 61beb6f.

- The three in-node components were never blamed for a soft stop, and no fault episode of any length named them: dock_detector and dock_drive in 2.5 h, coverage_planner in
  3.0 sim h (1.16 h of them active; the AMCL rows have no in-node component). Each also beat through the stationary/idle phases. This includes the dock detector, where a 1.8 s window is above the 0.76 s largest age.
- The reactive-executor stops are not an M9 effect: same blame, phase ("en route") and rate as the M7 proposal-config rows (M7: 59 and 91 per active hour; here 12, 8, 12 per session, 90 per active hour, about 70 with the
  recon's sweep time). They stay open from M7/M8 and are not solved by the in-node beat.
- Soft stops "explained" by harness actions: the dock stops follow the teleport (`set_pose`) between attempts as in M7; this classification is the one `summarize.py` has used since M7 (0.0-0.5 s window).
- Host load (1-min average sampled every 10 s, median of session medians, max): dock 15.5 / 22.0, coverage nav2 15.6 / 29.9, reactive 15.7 / 22.9, AMCL nav 3.7 / 15.3 of 12 logical cores (two instances at a time; M7: 7-10 busy cores).
  The AMCL nav sessions ran after the other instance's last session (the first one overlapped its end by about 2 minutes).
- The stand-in's own soft-stop counter delta equals the recorder's count in every session except reactive session 3 (13 vs 12): the extra stop came after the measured
  window, when the worker stopped the planner at the session end (`last_stack`: `coverage_planner` stale). The nav2 session started 23:50 ended in the same aggregator fault
  with the stand-in not yet stopped when read. A teardown effect of the harness, outside the window, not counted.
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
- Fault trials ran with the other instance busy (the dock trials beside the reactive recon `m9_recon_retry2`, the nav2 and reactive coverage trials beside each other); host load per
  row (median of the per-trial medians of the 1-min average) 7.4-31.9, max 36.7 of 12 logical cores; M7/M8 had the same arrangement.
- Aborted by my SIGTERM: one dock kill trial (invalid, not counted). Invalid recon sessions listed in section 2.
- The reactive-executor false-stop rate (90 per active hour, about 70 with the recon's sweep time) is in the M7 range (59 and 91) and unrelated to the beats; the nav2-executor
  soak has 0.81 active h (1.5 sim h) and bounds the false-stop rate only to about 3.7 per active hour at 95%.
- Beat placement in the planner counts waiting/idle ticks as work; a planner that is alive but cannot advance (no pose, no map) is not detected by the beat.
- Not done: in-node beats for Nav2 and slam_toolbox (third-party, watcher fallback stays), real-hardware timing.
- On the robot there is no relay and every process reads the system clock: the detection mechanics are the same (beats stop, the component is stale after max_age), the
  relay's clamp was never used here (`clamped=0`), and the equal-tick artefact of section 3 cannot occur. What does not carry over: the beat periods (LiDAR scan rate, fit
  and planning time on the robot's CPU), the stop distances, and the false-trip rates, which are sim numbers.

## 8. M9b: no relay (2026-11-21), supersedes the relay as the default path

Why: the review found that the negative ages were a float-rounding artefact of the patch (`ns * 1e-9` against the aggregator's `ns / 1e9`), not `/clock` skew (section 3). The patch is
fixed (oomwoo-ros2-tools 9c316cc, cbada42, 8f7bef7; tree identical to cb088fd), so the relay is not needed. In the kit it is now opt-in (`--beat-relay`, with `--health-clock wall`; the
M9 data above were taken with it) and off by default; nothing in M9b uses it. M9b runs: overlay built from 8f7bef7 (`tools_commit` in each record), nodes publish straight to
`/oomwoo/health/component`, aggregator, watcher and stand-in on sim time (`--health-clock sim`), CPU rendering, two instances at a time, labels `m9b_faults` / `m9b_soak`, raw data
`results/fullstack/m9b/` (published copies `m9b/published/`, chains in `m9b/_chains/`), kit commit 5ca79f7 or later (see records). `max_age_sec`: dock_detector 1.8 s, dock_drive 0.15 s,
nav2-executor planner 0.6 s (`nav_rosters_amcl_inode.yaml`, `nav_rosters_dock_inode.yaml`, unchanged), reactive-executor planner **0.8 s** (new `nav_rosters_amcl_reactive_inode.yaml`; the clean
recon had a largest age before refresh of 0.400 s, the review's loaded trace 0.70 s, so the rule's 0.6 s had no margin).

Commands (every sim.sh call with `OHK_RES_SUB=m9b OHK_OVERLAY=1`):

    tools/sim.sh overlay-build ; tools/sim.sh up 2
    nohup tools/m9_chain.sh 1 results/fullstack/m9b/_chains/faults_inst1.txt   # lines: --mode faults --reps N --health-sources watcher --health-clock sim --task T --faults F --label m9b_faults
    nohup tools/m9_chain.sh 2 results/fullstack/m9b/_chains/faults_inst2.txt   # started 60 s apart; then soak_inst{1,2}.txt (--sessions N --task T --duration-sim S --health-clock sim --label m9b_soak)
    python3 bench/fullstack/collect_m9.py --src results/fullstack/m9b --out results/fullstack/m9b/published
    python3 bench/fullstack/summarize.py --by-task results/fullstack/m9b/published/soak_m9b_*.jsonl
    python3 bench/fullstack/summarize_faults.py results/fullstack/m9b/published/faults_m9b.jsonl --label=m9b_faults

### Fault subset (kills only), no relay vs relay (section 4)

| Fault | n (no relay / relay) | Detected | Zero at the gate s, min / med / max, no relay | Relay (section 4) | Distance m no relay (relay) | Blamed |
|---|---|---|---|---|---|---|
| SIGKILL dock_detector | 10 / 20 | 10/10 (20/20) | 1.40 / 1.56 / 1.80 (see note) | 1.26 / 1.55 / 1.88 | 0.08-0.11 (0.07-0.11) | dock_detector |
| SIGKILL dock_drive | 5 / 10 | 5/5 (10/10) | 0.22 / 0.24 / 0.27 | 0.23 / 0.25 / 0.30 | 0.01-0.02 (0.01-0.02) | dock_drive + drive_supervision |
| SIGKILL coverage_planner, nav2 executor | 10 / 10 | 10/10 (10/10) | 0.55 / 0.63 / 0.72 | 0.54 / 0.62 / 0.72 | 0.12-0.15 (0.12-0.15) | coverage_planner |
| SIGKILL coverage_planner, reactive (0.8 s roster) | 5 / 10 | 5/5 (10/10) | 0.42 / 0.44 / 0.55 | 0.39 / 0.43 / 0.57 | 0.17-0.22 (0.16-0.23) | coverage_planner + drive_supervision |

Aggregator first fault (s, min/med/max, no relay vs relay): detector 1.30 / 1.46 / 1.70 vs 1.16 / 1.45 / 1.77; dock_drive 0.11 / 0.13 / 0.17 vs 0.12 / 0.14 / 0.19; nav2 planner 0.45 / 0.53 / 0.61 vs 0.43 / 0.51 / 0.62; reactive
0.31 / 0.33 / 0.44 vs 0.28 / 0.33 / 0.46. All 30 trials valid and exposed, robot stopped in all; rotation after the fault 0.0-8.1 degrees (relay 0.0-8.3). Note on dock_detector kill rep 10: the stand-in
soft-stopped at 1.221 s, 0.4 s BEFORE the aggregator's first fault (1.621 s; a heartbeat gap, stand-in back in RUN at 1.321 s); the stop caused by the detector is the second one at 1.731 s, and
that value is used in the table (the raw "gated first zero" of that trial is 1.221 s, which would make the minimum 1.22). The 1.8 s window and the relay's data agree: the in-node stops come at the roster value plus the time since the last beat, relay or not.
Freeze (SIGSTOP) rows were not repeated; the relay-run rows (10/10 detector, 5/5 planner nav2 and reactive, 5/5 dock_drive) stand as the freeze evidence (a stopped node produces the same silence as a killed one).

### Soak without relay (4.28 sim h, 13 sessions: dock 4 x 1800 s, nav2 coverage 2 x 1800 s, reactive coverage 4 x 700 s, AMCL navigation 1 x 1800 s; all valid)

| Task | Sim h | Active h | Soft stops | Blamed on an in-node component | Blamed `drive_supervision` | No aggregator episode ("unknown" class) | Per active hour (all stops) | Relay run (section 5), same fixed active-time accounting |
|---|---|---|---|---|---|---|---|---|
| dock | 2.00 | 2.00 | 23 | **0** | 20 (all 0.2-0.3 s after a harness `set_pose`) | 3 | 11.5 | 33 in 2.5 h, 0 in-node, 0 unexplained |
| coverage, nav2 executor | 1.00 | 0.76 | 1 | **0** | 0 | 1 | 1.3 | 0 in 0.81 active h |
| coverage, reactive executor | 0.78 | **0.48** | 42 | **0** | 41 | 1 | 88 | 32 in 0.35 active h (90/h) |
| AMCL navigation | 0.50 | 0.50 | 0 | 0 | 0 | 0 | 0 | 0 in 1.0 h |
| Total | 4.28 | 3.74 | 66 | **0** | 61 | 5 | | 65 stops, 0 in-node |

- **Stops blamed on in-node components: 0 of 66 without relay (0 of 65 with relay)**; no aggregator episode of any length named dock_detector, dock_drive or coverage_planner in either run.
- The 61 `drive_supervision` stops are the same ones as with the relay (dock: the harness teleport; reactive: the 5 Hz command stream against the 0.30 s window, 88 vs 90 per active hour).
- 5 stops are new: the stand-in soft-stopped ("no heartbeat for 0.150 s") with no aggregator fault episode and the recorder's last MCU heartbeat 0-1 ms old (one reactive stop 150 ms), i.e. delivery gaps on the stand-in side. They happened at host load
  26-31 (1-min average of 12 logical cores; relay run: 15-16 in the soaks). Sim-time health nodes cost about one core per instance more: container CPU 3.76 / 4.51 / 4.50 / 3.43 cores (dock / nav2 / reactive / AMCL) against 3.03 / 3.73 / 3.70 / 2.20
  with the wall-clock health stack, worst 30 s RTF window 0.965-0.999 (relay run 0.981-0.999; all sessions passed the harness RTF validity checks (session RTF 0.973-0.9996, every 30 s window at least 0.965)). The load is the likely cause of the 5 stops; this was not isolated or tested. They are not blamed on any component. They are counted as unexplained stops: 5 in 4.28 h (1.2 per sim hour); the relay run had none in 6.5 h.
- Active time is as in section 5 (`summarize.py` at 61beb6f). The reactive row has 0.48 active h, short of the 0.5 h that was asked for (4 x 700 s sessions, the sweep ends after about 400-480 s).

### What M9b supersedes

- Section 3's relay as the way the M9 numbers were taken and its mechanism claim: the relay is not needed; section 3's evidence stays as the explanation of the first smoke runs. The no-relay rows above reproduce the relay rows within the spread.
- Reactive planner `max_age_sec` 0.6 s (section 2, sections 4-5, `nav_rosters_amcl_inode.yaml` with `coverage_reactive_inode` in the relay run): now 0.8 s for the reactive executor. The reactive kill rows were re-run (0.42-0.55 s, 5/5).
- Container CPU and RTF figures of section 5: measured with the wall-clock health stack; with sim-time health nodes (as on the robot, one clock) the simulation costs about one core more per instance.
- Not superseded: the recon periods (nodes unchanged), the freeze rows, the n=20/10/15 detector, drive and planner counts, the relay-run soak rows as the 6.5 h bound.
