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
    tools/sim.sh run 2 --mode faults --faults sigstop_nav2,sigkill_nav2,sigkill_slam,sigkill_bridge,pause_gz --reps 5 --health-sources watcher
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
- `health_events`: per aggregator fault episode and per soft stop: `goal`, `phase` (start / en route / end / recovery / between),
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
- `standin_soft_stop_s`, `standin_soft_stop_reason`, `standin_run_from_signal_s` (stand-in back in RUN, may be before the resume).
- `standstill_s`, `robot_end_state` (stopped | frozen | moving), `distance_to_standstill_m`, `distance_total_to_end_m`: from
  the simulator's own `gz /odom` (physics ground truth, independent of the ROS bridge). pause_gz freezes physics, so "stopped"
  there only reflects the commands after the unpause.
- `recovered_by_itself`, `recovered_gated_nonzero_s` (from the resume, kills: from the signal), `goals_after_signal`, `detected`.
- `sim_clock_advance_during_hold_s`: Gazebo clock progress while held (about 0 for pause_gz).

`n4_respawn.py` measures how long the writer guard keeps a component down after SIGKILL + respawn of the bridge.

## Sim vs wall
Gazebo `/clock` = sim; everything stamped `wall`/`monotonic` = host. At RTF 1.0 (the only supported setting for timing claims;
`timing_valid`) they agree. Health nodes run on wall time by default (see docs/SPEC.md M6c); header stamps in the stack are sim time.
