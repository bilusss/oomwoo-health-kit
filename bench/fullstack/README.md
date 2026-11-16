# bench/fullstack: health loop on the real OOMWOO sim stack (M6b/M6c)

Gazebo + Nav2 + slam_toolbox in containers `ohk-sim-<i>` (RTF 1.0, max 4 at once), MCU stand-in on the last cmd_vel hop
(bridge subscribes `cmd_vel_gated`), aggregator, heartbeat sources `fake` or `watcher` (real topic watcher, static roster,
`config/nav_watch_sim.yaml` + `config/nav_rosters.yaml`). Run `tools/sim.sh` without arguments for the fleet commands.

## Run
    tools/sim.sh build                      # after changing kit python code (configs and bench/ are read from the mounted source)
    tools/sim.sh up 4                       # containers, domains 41..44
    # soak, watcher sources, 15 min, mixed workload (default clock = wall):
    tools/sim.sh run 1 --health-sources watcher --workload mix --duration-sim 900 --profile default
    # faults: one stack per trial, 5 repetitions of each fault (about 2.5 min per trial):
    tools/sim.sh run 2 --mode faults --faults sigstop_nav2,sigkill_nav2,sigkill_slam,sigkill_bridge,pause_gz,sigkill_watcher,sigkill_aggregator --reps 5 --health-sources watcher
    tools/sim.sh status | tools/sim.sh stop <i|all> | tools/sim.sh down
    --sessions 0 = endless; --quiet-window "2026-11-14T08:00/2026-11-14T22:00" (local time) keeps sessions out of that window.
    python3 bench/fullstack/probe_rates.py --seconds 120   # rates of the watched signals, moving vs stationary (inside a container)

## Files (results/fullstack/<i>/)
`sessions.jsonl` one line per soak session; `faults.jsonl` one line per fault trial (a fault session is NOT a soak session);
`run.log` events; `progress.json` live; `stack_<n>.log` the stack output.

## Soak line: main fields
- `valid`, `invalid_reason`, `checks`: see run.py docstring. `health_clock` wall|sim, `health_clock_note`, `workload` plain|mix.
- `goals` (per goal: result, `duration_wall_s`/`duration_sim_s`, `distance_odom_m`, `soft_stops`, `fault_episodes`), `goals_by_result`.
- `soft_stops` / `soft_stops_n`: stand-in transitions into soft_stop (wall + sim stamp, reason, aggregator state around it).
- `standin_soft_stops_at_start/_end/_delta` (the stand-in's own cumulative counter) and `aggregator_at_end.healthy_for_sec`: two counters independent of the recorder.
- `health_events`: per NON-EMITTING aggregator episode (any period with emit_mcu_heartbeat false: fault, arming, no_roster; `states`) and per soft stop; a soft stop has `class`: detection (an episode naming a critical component overlaps it), delivery_gap (aggregator healthy, recorder saw a heartbeat gap) or unknown. Fields: `goal`, `phase` (start / en route / end / recovery / between),
  `blamed` (components in missing/stale/unhealthy_critical), `duration_ms`, `caused_soft_stop`, and at fault start
  `odom_v`, `odom_w` (robot motion), `cmd_age_wall_s` (age of the last /cmd_vel), `last_cmd_nonzero`.
  `health_event_tables`: counts by phase and "phase / component". Phase windows in `phase_windows_s` (bench/fullstack/phases.py:
  start = first 3 s of a goal; end = from 1 s before to 2 s after the result; recovery = spin/backup/wait/drive_on_heading action
  active, plus 1 s; the end window of the previous goal wins over the next goal's start).
- `uncommanded_motion`: odom above the drive_supervision thresholds with no /cmd_vel newer than 0.15 s (events, durations; `last_cmd_nonzero` False = coast after a zero setpoint).
- `hb_gaps_over_200ms`, `hb_gaps_n`: gaps between mcu_heartbeat messages seen by a separate recorder process (wall).
- `rtf`, `rtf_30s_windows`, `cpu_cores_container`, `cpu_cores_by_process`, `distance_m` (path from /odom = ground truth).

## Fault line: main fields (all `*_s` are seconds from the signal, host monotonic clock = sim seconds at RTF 1.0)
- `fault`, `rep`, `signal`, `target_pid`, `signal_sim_s` (Gazebo clock at the signal), `speed_at_signal_mps`, `hold_s`, `observe_after_resume_s`.
- `gated_first_zero_s`: first zero Twist on cmd_vel_gated after the signal. `gated_nonzero_after_zero_before_resume`: must be 0.
- `aggregator_first_fault_s`, `blamed` (union over the first fault episode), `blamed_first_sample`.
- `standin_soft_stop_s`, `standin_soft_stop_reason`, `standin_run_from_signal_s` (stand-in back in RUN, may be before the resume). pause_gz: the fault instant is the last advance of the /clock value before it stalled (`signal_instant_basis`), not the CLI call.
- `truth_track` [t_rel, x, y, speed, yaw rate, yaw] every ~0.25 s from the gz ground truth.
- `standstill_s`, `robot_end_state` (stopped | frozen | moving), `distance_to_standstill_m`, `distance_total_to_end_m`: from
  the simulator's own `gz /odom` (physics ground truth, independent of the ROS bridge). pause_gz freezes physics, so "stopped"
  there only reflects the commands after the unpause.
- sigkill_bridge: the gate (stand-in) sits BEFORE ros_gz_bridge, the only path to the simulated motors. The loop detects the kill
  and the stand-in outputs zero, but the zero cannot be delivered; the diff-drive keeps its last velocity. The distance/time in that row
  is NOT stopping performance. On the robot the MCU is the last hop; loss of the link to the MCU was not tested.
- `recovered_by_itself`, `recovered_gated_nonzero_s` (from the resume, kills: from the signal), `goals_after_signal`, `detected`.
- `sim_clock_advance_during_hold_s`: Gazebo clock progress while held (about 0 for pause_gz).

`summarize.py sessions.jsonl...` prints the goal accounting (succeeded / cancelled mid-drive / cancelled early / unplannable / cut by session end / recovery probes) and soft stops by class and phase. `n4_respawn.py` measures how long the writer guard keeps a component down after SIGKILL + respawn of the bridge.

## Sim vs wall
Gazebo `/clock` = sim; everything stamped `wall`/`monotonic` = host. At RTF 1.0 (the only supported setting for timing claims;
`timing_valid`) they agree. Health nodes run on wall time by default (see docs/SPEC.md M6c); header stamps in the stack are sim time.

## M7 tasks (AMCL, coverage, docking, BT recoveries)
`--task` selects the stack and workload (`bench/fullstack/tasks.py`): `nav_slam` (default, M6), `nav_amcl`, `coverage`, `coverage_reactive`,
`coverage_reactive_cmd30` (proposal config), `dock`, `recovery`. `--composition false` runs one process per Nav2 node (needed for per-node
faults on the AMCL tasks). Use `OHK_RES_SUB=m7` (exported before `tools/sim.sh up|run|status`) to keep new raw data in `results/fullstack/m7/<i>/`.

    export OHK_RES_SUB=m7; tools/sim.sh down; tools/sim.sh up 3
    tools/sim.sh run 1 --health-sources watcher --task nav_amcl --workload mix --duration-sim 1800 --sessions 4 --profile default
    tools/sim.sh run 2 --health-sources watcher --task coverage --duration-sim 1800 --sessions 4 --profile shm_hc60
    tools/sim.sh run 3 --health-sources watcher --task dock --duration-sim 1800 --sessions 2 --profile default
    tools/sim.sh run 3 --health-sources watcher --task recovery --duration-sim 1800 --sessions 4 --profile default
    tools/sim.sh run 1 --mode faults --reps 5 --health-sources watcher --task nav_amcl --composition false --faults sigkill_amcl,sigstop_amcl,sigkill_controller,sigstop_controller
    tools/sim.sh run 2 --mode faults --reps 5 --health-sources watcher --task dock --faults sigkill_dock_drive,sigstop_dock_drive,sigkill_dock_detector,sigstop_dock_detector
    tools/sim.sh run 3 --mode faults --reps 5 --health-sources watcher --task recovery --composition false --faults sigkill_behavior,sigstop_behavior
    tools/sim.sh run 1 --mode faults --reps 5 --health-sources watcher --task coverage_reactive_cmd30 --faults sigkill_coverage_fast,sigstop_coverage_fast
    # tables (published ones: results/fullstack/soak_m7_*_kitwip.jsonl and faults_m7_*_kitwip.jsonl)
    python3 bench/fullstack/summarize.py --by-task results/fullstack/soak_m7_*.jsonl          # sim hours AND active hours, stops per active hour
    python3 bench/fullstack/summarize_faults.py results/fullstack/faults_m7_*.jsonl --latest-label-per-task    # the fault table in FINDINGS
    python3 bench/fullstack/summarize_faults.py results/fullstack/faults_m7_*.jsonl --label=m7_faults --task=recovery --detail   # earlier series

Coverage jobs finish their sweep and then idle, so `--by-task` reports active time (`active_seconds`: up to the last motion evidence in the
session record) next to sim time and uses it for the per-hour rates. Fault trials are summarized over EXPOSED trials (definition in the
`summarize_faults.py` docstring, set after the trials were seen). `OHK_RUN_STAGGER_S=<s>` spaces out instance starts; sessions and fault trials
record `loadavg_series` (host 1-min load every 10 s).

Live-graph checks before writing a watcher config (inside a container with the stack up): `python3 bench/fullstack/graph_facts.py`
(publishers, subscriptions, QoS) and `probe_rates.py --seconds 100 [--extra /topic:pkg/msg/Type]` (rates moving / stationary).
Workloads (`jobs.py`): coverage = upstream `coverage_planner` started after arming; dock = cycle of start poses with a harness `set_pose` between
attempts (its records carry `harness_actions`; `summarize.py --by-task` reports soft stops within 2 s of one); recovery = goals with a wall spawned
across the path (persistent or removed after 8 s) so that bt_navigator's own recovery branch runs. The gz service calls use the python
gz-transport bindings (the `gz` CLI stalled /odom and /cmd_vel). Fault triggers (`when`): moving, fast (>= 0.30 m/s), recovery, dock_back.

## M8 (command-chain and dock-fix rules; at most 2 instances)
New watcher rules (opt-in, `docs/SPEC.md` M8, `docs/M8_RECON.md`) and their bench tasks: `nav_amcl_chain`, `recovery_chain` (soaks; rule A, `nav_watch_amcl_chain.yaml` +
`nav_rosters_amcl_chain.yaml`), `recovery_probe_chain` / `recovery_probe` (fault trials for behavior_server with the rule / with the M7 watch set as control;
workload `jobs.RecoveryProbeJob` = NavigateToPose to an open pose, then a 3.14 rad spin and a 0.45 m backup at 0.15 m/s sent directly, so that the nominal rate lasts
seconds), `dock_fix` (rule B, `nav_watch_dock_fix.yaml` + `nav_rosters_dock_fix.yaml`). Fault `sigkill_behavior_nominal` / `sigstop_behavior_nominal` is signalled only while a
recovery action runs at its nominal rate (spin >= 0.9 rad/s or backup >= 0.10 m/s): the exposure criterion of `summarize_faults.py`, written down before the runs.
`run.py` writes one stack log per session or trial and run: `results/fullstack/m8/<i>/stack_<run start>_<task>_<soak|faults>_<n>.log` (no overwriting); records name it in
`stack_log`. `OHK_RUN_STAGGER_S` only spaces the instances of ONE `tools/sim.sh run all` call. `loadavg_series` (host 1-min load average every 10 s) is in every soak session and
fault trial and is summarised by `summarize.py --by-task` and `summarize_faults.py`.

    # recon: traces of the message streams while a stack and a workload run (inside the container; see m8_analysis.py)
    python3 bench/fullstack/probe_trace.py --seconds 600 --out /results/trace.jsonl
    python3 bench/fullstack/m8_analysis.py chain|dock|derive|replay-chain|replay-dock|reactive TRACE...
    python3 bench/fullstack/m8_stops.py results/fullstack/soak_m8_*.jsonl      # every soft stop with the watcher's own reason from the retained stack log
    # runs as executed (OHK_RES_SUB=m8 keeps the raw data in results/fullstack/m8/<i>/)
    export OHK_RES_SUB=m8; tools/sim.sh down; tools/sim.sh up 2
    tools/sim.sh run 1 --mode faults --reps 12 --health-sources watcher --task recovery_probe_chain --composition false --faults sigkill_behavior_nominal,sigstop_behavior_nominal --label m8_faults
    tools/sim.sh run 2 --mode faults --reps 10 --health-sources watcher --task nav_amcl_chain --composition false --faults sigkill_controller,sigstop_controller --label m8_faults
    tools/sim.sh run 2 --mode faults --reps 3 --health-sources watcher --task nav_amcl_chain --composition false --faults sigkill_controller --label m8_faults   # replaces one trial that did not arm
    tools/sim.sh run 2 --mode faults --reps 10 --health-sources watcher --task dock_fix --faults sigkill_dock_detector,sigstop_dock_detector --label m8_faults
    tools/sim.sh run 1 --mode faults --reps 10 --health-sources watcher --task recovery_probe --composition false --faults sigkill_behavior_nominal,sigstop_behavior_nominal --label m8_faults_control
    tools/sim.sh run 1 --mode faults --reps 10 --health-sources watcher --task nav_amcl --composition false --faults sigkill_controller,sigstop_controller --label m8_faults_control
    tools/sim.sh run 2 --health-sources watcher --task dock_fix --duration-sim 1800 --sessions 2 --profile default --label m8_soak
    tools/sim.sh run 1 --health-sources watcher --task recovery_chain --duration-sim 1800 --sessions 4 --profile default --label m8_soak      # started together with the next line
    tools/sim.sh run 2 --health-sources watcher --task nav_amcl_chain --workload mix --duration-sim 1800 --sessions 4 --profile default --label m8_soak
    # item D, one instance only, nothing else running
    tools/sim.sh run 1 --health-sources watcher --task coverage_reactive --duration-sim 700 --sessions 2 --profile default --label m8_reactive    # run twice (4 sessions)
    docker exec -d ohk-sim-1 ...probe_trace.py --seconds 2000 --out /results/trace_reactive.jsonl     # started right after each run; copied to results/fullstack/m8_recon/
    python3 bench/fullstack/m8_analysis.py reactive results/fullstack/m8_recon/trace_reactive*.jsonl
    # sort the raw records into the published files, then print every table
    python3 bench/fullstack/collect_m8.py
    python3 bench/fullstack/summarize.py --by-task results/fullstack/soak_m8_*_2026-11-15_kitwip.jsonl
    python3 bench/fullstack/summarize_faults.py results/fullstack/faults_m8_2026-11-15_kitwip.jsonl --label=m8_faults          # the rule under test
    python3 bench/fullstack/summarize_faults.py results/fullstack/faults_m8_2026-11-15_kitwip.jsonl --label=m8_faults_control  # M7 watch set, same trials
    python3 bench/fullstack/summarize_faults.py results/fullstack/faults_m7_2026-11-14_kitwip.jsonl --latest-label-per-task    # the M7 baseline rows
