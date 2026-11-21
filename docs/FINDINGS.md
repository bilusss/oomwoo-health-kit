# M9 (2026-11-18 to 21): in-node work-path heartbeats in OOMWOO's own nodes

Following the maintainer's answer in makerspet/oomwoo#84 (own nodes send in-node work-path heartbeats; topic watchers are the fallback for third-party nodes), the opt-in package
`oomwoo_health_beat` and beats in `dock_detector`, `dock_drive` and `coverage_planner` were written as a patch to makerspet/oomwoo-ros2-tools (branch `health-in-node-heartbeats`,
9c316cc, cbada42, 8f7bef7). Every number, the method and the limits are in `docs/M9_RECON.md`; raw data in `results/fullstack/m9/` (wall-clock health stack with a relay) and
`results/fullstack/m9b/` (everything on sim time, no relay, as on the robot: section 8, the default path now).

| Fault (SIGKILL) | M7/M8 | M9b, no relay | Zero at the gate (s) | Distance |
|---|---|---|---|---|
| `dock_detector` while reversing | M7 0/10 (kill + freeze); M8 `reverse_fix` 20/20 at 1.50-2.11 s | 10/10 (relay run 20/20 kill, 10/10 freeze) | 1.40-1.80 | 0.08-0.11 m |
| `coverage_planner`, nav2 executor | M7 0/10, the controller finishes the waypoint (0.5 m) | 10/10 (relay run 10/10 kill, 5/5 freeze) | 0.55-0.72 | 0.12-0.15 m |
| `coverage_planner`, reactive | M7 detected | 5/5, but `drive_supervision` trips first; the planner is named at 0.68-0.79 s | 0.42-0.55 | 0.17-0.22 m |
| `dock_drive` | M7 detected, 0.35-0.41 s | 5/5 | 0.22-0.27 | 0.01-0.02 m |

- `max_age_sec` from the measured beat periods: dock_detector 1.8 s (one fit every ~0.6 s, so this is the slowest stop; it is no faster than M8's `reverse_fix`, but has no false stops),
  dock_drive 0.15 s, planner 0.6 s (nav2 executor) and 0.8 s (reactive).
- False stops blamed on an in-node component: **0 of 66 in 4.28 sim h** (no relay) and 0 of 65 in 6.5 sim h (relay). The other stops are known: `drive_supervision` after harness
  teleports (dock) and on the reactive executor's 5 Hz command stream (88-90 per active hour, as in M7), plus 5 with no aggregator fault: the sim clock stalled ~100 ms while the
  stand-in times out on the steady clock (a sim-only clock-domain effect).
- Found on the way: the aggregator counts any negative age as stale (`core.py` line 131), so a producer converting `ns * 1e-9` instead of the monitor's `ns / 1e9` is "stale" on
  about 4 in 10 shared sim-clock ticks (28 soft stops in 240 sim s even with `max_age_sec` 5 s). The patch uses `/ 1e9`; reported to the aggregator.
- Firmware policy (makerspet/oomwoo-firmware#3, 2026-11-20): heartbeat loss makes motor outputs safe and keeps the shared motor/LiDAR rail on; only the RC watchdog and hard
  faults cut it. The kit's MCU stand-in already behaves this way.
- Limits: simulation only, one 6-core PC, small fault samples; the beats prove the loop runs, not that its output is right; Nav2 and slam_toolbox keep the watcher fallback.

# M8 (2026-11-15 and 16): what the watcher alone can still close of the M7 gaps

Raw data: `results/fullstack/faults_m8_2026-11-15_kitwip.jsonl` (fault trials), `results/fullstack/soak_m8_<task>_2026-11-15_kitwip.jsonl` (soak sessions; the two item-D runs on 2026-11-16 carry the same tag),
`results/fullstack/m8/<i>/` (per-instance run logs and **every stack log, one file per session or trial**: `stack_<run start>_<task>_<soak|faults>_<n>.log`; the record's `stack_log` names it and `instance` the directory),
`results/fullstack/m8_recon/` (message traces and recon sessions), `results/fullstack/_m8_smoke/` (smoke runs and the first fault attempt, not counted). Kit: dirty tree on top of
`f5dd15e` (`kitwip`), aggregator `b2b9796`. Recon, graph facts and the derivation of the bounds: `docs/M8_RECON.md`. Every table below is printed by the commands at the end of
`bench/fullstack/README.md` (M8). Same setup as M6/M7 (software GL, RTF 1.0, wall-time health nodes, 150 ms soft stop), **at most 2 instances in parallel**.

**Provenance.** The records carry no hash of the watch/roster files (`run.py` now records `config_sha256` for future runs). The two chain and dock configs were written before the first trial and their
bounds were never changed; their file times predate every trial. Afterwards only comments (and, below, nothing else in the YAML) were edited. The analysis scripts (`summarize*.py`, `m8_analysis.py`) and, after the
measurements, the rule code (see "Changes after the measurements") were edited; the tree is dirty. The rotation figures of M7 and M8 were recomputed after a review (see "Rotation").

## Result in one table
| Gap | Rule (opt-in watch type) | Verdict | Key numbers (exposed trials, default profile) |
|---|---|---|---|
| A. commander behind Nav2's velocity smoother stops commanding while the smoother keeps replaying the setpoint | `command_chain` on `/cmd_vel_nav` -> `/cmd_vel_smoothed`, `in_max_age_s` 0.35 | **partly closed**: stale motion is stopped in 24 of 24 trials, in about half the time | behavior_server spin: zero at the gate 0.53-0.64 s (same trials, M7 watch set: 1.26-1.34 s, 0 of 10 SIGKILL ever stopped by the loop), rotation after the signal median 32 degrees (control: 64) |
| B. dock detector stops while `dock_drive` reverses | `reverse_fix` on `/cmd_vel` + `/dock_detector/dock_pose`, `fix_max_age_s` 1.85 | **partly closed**: 20 of 20 stopped, coast shortened | zero at the gate 1.50-2.11 s (M7: 2.61-3.00 s, 0 of 10), reversing 0.09-0.13 m (M7: 0.16-0.18 m); 2 of 216 soak attempts (95% CI 0.1-3.3%) got a soft stop |
| C. nav2-executor coverage planner killed | none | **not closed, by design** | the controller finishes the waypoint (M7: 6.0-6.2 s, 0.5 m); not a motion-safety matter; nothing built |
| D. reactive planner command period | none (measurement) | measured, with one instance running | period 0.200 s p50; a missed 5 Hz tick in 7.2% of the periods (n=3597, 12.9 min) |

False trips with the rules enabled: **0 soft stops in 4.0 sim hours (recovery_chain 2.0 h, nav_amcl_chain 2.0 h; both composed Nav2)**; dock: 2 `dock_fix` stops in 2.0 sim hours (216 attempts), both in the shm_hc60 profile.

## What was built (details in `docs/SPEC.md` M8)
Two ROS-free cores in `topic_watch.py` (`CommandChainGuard`, `ReverseFixGuard`), two thin rclpy adapters, two YAML watch types (`command_chain`, `reverse_fix`), configs
`nav_watch_amcl_chain.yaml` / `nav_rosters_amcl_chain.yaml` and `nav_watch_dock_fix.yaml` / `nav_rosters_dock_fix.yaml`. Nothing upstream was changed. No M6/M7 config selects the rules.
Fail-safe cases (non-finite value, backwards clock, no input ever seen while the stage replays, no fix ever seen while reversing) are unit-tested; launch tests `u1`-`u8` give a fault and a
no-fault case per rule, a control showing the M7 watch set does not see the replayed setpoint, and the two "must not switch off silently" cases below.
**Scope of rule B.** "Reversing" is inferred from the held command of ANY writer on `/cmd_vel`. `dock_drive` must be the only reverser on that stream: Nav2 BackUp (about 3 s at -0.1..-0.15 m/s) or teleop would trip the rule
1.85 s into the reverse, because no dock fix exists (unit test `test_reverse_by_any_writer_trips_without_a_fix`). Only `nav_watch_dock_fix.yaml` (task `dock_fix`, no Nav2) selects it (test `test_only_the_dock_config_selects_rule_b`).

## Changes after the measurements (review)
After the soaks and trials, the rule code was changed so that rule A cannot switch itself off silently: a message-type option per watch (`msg_type` / `cmd_type`, Twist or TwistStamped), a graph check (both stage topics need a
publisher of that type, else the component is not-ok with the reason in the watcher log; for rule B a command topic whose publishers have another type), and "odometry above 0.02 m/s / 0.05 rad/s for more than 0.5 s while the stage
output was NEVER seen" is not-ok. **All measurements below were taken with the code before this change.** The change acts only in states in which the old code said "healthy because nothing was ever seen / nothing was checked":
(a) a stage topic without a publisher or of another type, (b) motion with no output ever. Evidence that the measured runs did not depend on those states: every `command_chain: no heartbeat` warning in the 46 fault-trial stack logs of the
rule is the "no non-zero input newer than ..." path of the original code, and the 4.0 h of soak logs contain none; the types and publishers on the stage topics are Twist and present in `graph_nav_amcl.json`. I did not re-run anything, so the
claim that the numbers would be the same rests on this reasoning and on the container tests (launch tests `u1`-`u8` and the unit tests pass with the new code; `u7` and `u8` exercise the new paths). One known side effect: a stage that disappears from the graph
(for example during Nav2's own lifecycle reset) is now not-ok at once, which only matters in a state that is already a fault.

## Method
1. **Recon first** (`docs/M8_RECON.md`): message traces on the live stacks (469 s of the BT recovery task, 2916 s / 89 reversing episodes of the dock task). `behavior_server` writes the
   same input topic as the controller (`/cmd_vel_nav`: controller 5 Hz, BT spin and backup 10 Hz, wait nothing; the rate during DIRECT spin actions, which the fault trials use, was not traced), so one pair of topics covers both commanders.
   `/dock_drive/state` is published only on change, so reversing is inferred from the held command (linear x <= -0.02 m/s).
2. **Bounds from the traces, by formulas written down before any trial or soak** (`m8_analysis.py derive` prints them): `in_max_age_s` = ceil_50ms(1.5 x longest non-zero input period 0.201 s) = 0.35;
   `stop_grace_s` = ceil_50ms(1.5 x longest zero-input-to-zero-output time 0.344 s) = 0.55; `fix_max_age_s` = ceil_50ms(1.5 x p99 of the fix gaps inside reversing episodes 1.202 s) = 1.85. **A uses 1.5 x the maximum, B 1.5 x the p99: this was a choice**
   (1.5 x the maximum fix gap, 2.403 s, would be 3.65 s, beyond dock_drive's own 3.0 s hold). The fix gaps come in steps of the 0.6 s period, so 1.85 s sits 50 ms above three periods. No bound was changed after a trial or soak; no second variant was run.
3. **Exposure criteria, written down in `summarize_faults.py` before the first M8 trial.** behavior_server: signal only while a spin runs at >= 0.9 rad/s or a backup at >= 0.10 m/s (trigger
   `recovery_nominal`, 0.2 s dwell); controller_server: moving >= 0.12 m/s for 2 s; dock_detector: dock_drive in BACK, moving >= 0.03 m/s for 2 s. Invalid trials (did not arm, no trigger in 150 s) are not
   counted; a series needs >= 10 exposed trials per fault. Exposed trials: behavior 12 + 12 (rule) and 10 + 10 (control), controller 12 + 10 (rule) and **9** + 10 (control: one SIGKILL trial did not arm and was not repeated, so
   that control cell is below the 10 required), dock detector 10 + 10. The rule's controller SIGKILL cell has 3 extra repetitions (reps 11-13) run to replace one trial (rep 10) that did not arm.
4. **Workload for behavior_server faults changed after one attempt.** The first attempt (task `recovery_chain`, BT recoveries with a wall) reached no spin at the nominal rate in 150 s (spins aborted
   after 0.2-0.3 s with "Collision Ahead - Exiting Spin"; `_m8_smoke/records_smoke.jsonl`, 1 invalid, 1 interrupted by me). All behavior trials therefore use `jobs.RecoveryProbeJob`: NavigateToPose to an open pose,
   then a direct 3.14 rad spin and a direct 0.45 m backup at 0.15 m/s. **In all 44 behavior trials the trigger fired in the spin; no backup was exposed.**
5. **Control series with the same workload, trigger and kit but the M7 watch set** (`recovery_probe`, `nav_amcl`; label `m8_faults_control`) so the comparison does not rest on M7's different trigger and load.
6. **Composition.** The fault trials of rule A (and the recon trace of the recovery task) ran non-composed Nav2 (`composition false`, one process per node, needed to signal one node). **The rule-A soaks ran the default composed launch
   (one `component_container`)**; there a crash of one node takes the container with it, so the per-node faults were never exercised under the rule in composed mode. The dock stack has no Nav2.
7. **Load**: the host 1-minute load average is recorded every 10 s (`loadavg_series`, works; shown per row below). It rises to 14-19 at the stack starts; the per-row medians over the trials are 6.8-9.8 (rule rows incl. dock) and 7.5-9.8 (control rows).
   In the soaks the median of the session medians is 4.5 (max 11.0) for the two parallel A soaks and 4.6 (max 11.4) for the dock soaks. `OHK_RUN_STAGGER_S` was exported but only spaces the instances of one `run all` call, which I did not use:
   the two A soaks started 15 s apart (11:37:56 and 11:38:11; the queue's own start of the second was refused because a control run had not exited yet), the two dock soaks together (13:39:47). Fault trials of one instance ran
   next to trials or recon of the other, never next to a soak; one 35 s dock_fix soak session that I stopped at once is in the soak file as invalid.

## Rotation (corrected)
`summarize_faults.py` first integrated |yaw rate| only up to the last ground-truth sample at or before standstill (samples are ~0.24 s apart), which under-reports. `rotation_deg` now takes the ground-truth yaw from `truth_track`,
unwraps it, interpolates at the signal and at the standstill time and sums the absolute changes (unit tests). All rotation figures below, **including the M7 rows**, come from the corrected function; the M7 text further down was corrected as well.
The rule/control ratio of the behavior_server rotation is 32/64, about 0.5.

## Gap A: fault trials (seconds from the signal; hold 5 s; behavior_server: spin, controller_server: moving 0.2 m/s)
Tables: `summarize_faults.py results/fullstack/faults_m8_2026-11-15_kitwip.jsonl --label=m8_faults` (rule) and `--label=m8_faults_control` (M7 watch set); M7 rows from `faults_m7_...`. All fault trials: non-composed Nav2, profile default.
"Detected" below means: the loop's stand-in soft-stopped the robot; with the rule that is a stop of **stale motion** (the smoother replaying a setpoint nobody refreshes), see the second point under "Reading the table".

| Fault | Series | Composition | Exposed | Stopped any time / within hold | Aggregator first fault (min/med/max s) | Zero at the gate (min/med/max s) | Distance to standstill (m) | Rotation after signal (deg, min/med/max) | Blamed |
|---|---|---|---|---|---|---|---|---|---|
| SIGKILL behavior_server | M8 rule | non-composed | 12 | 12 / 12 | 0.43 / 0.46 / 0.54 | 0.53 / 0.57 / 0.64 | 0.00 | 28.6 / 32.0 / 37.1 | command_chain |
| SIGKILL behavior_server | control (M7 set) | non-composed | 10 | **0 / 0** | - | 1.26 / 1.29 / 1.34 | 0.01 | 61.9 / 64.0 / 66.0 | none |
| SIGKILL behavior_server | M7 (BT, 5 exposed: 4 spins, 1 backup) | non-composed | 5 | 0 / 0 | - | 1.01 / 1.29 / 1.32 | 0.00-0.15 | spins 60.8-64.4, backup 0.0 | none |
| SIGSTOP behavior_server 5 s | M8 rule | non-composed | 12 | 12 / 12 | 0.43 / 0.47 / 0.52 | 0.53 / 0.57 / 0.63 | 0.00 | 29.1 / 31.5 / 36.6 | command_chain; 11 of 12 commands resumed after the SIGCONT |
| SIGSTOP behavior_server 5 s | control (M7 set) | non-composed | 10 | 10 / **0** | 17.97 / 18.45 / 19.26 | 1.27 / 1.28 / 1.30 | 0.01 | 62.6 / 63.7 / 64.3 | local_costmap, ~18 s after the signal (see below) |
| SIGSTOP behavior_server 5 s | M7 (2 exposed) | non-composed | 2 | 2 / 0 | 8.4 | 1.23 / 1.29 / 1.36 | 0.00 | 58.8 and 66.3 | local_costmap |
| SIGKILL controller_server | M8 rule | non-composed | 12 | 12 / 12 | 0.28 / 0.42 / 0.47 | 0.39 / 0.52 / 0.58 | 0.09 / 0.11 / 0.12 | 3.4 / 4.6 / 5.9 | command_chain + local_costmap (12 of 12 both) |
| SIGKILL controller_server | control (M7 set) | non-composed | **9** | 9 / 9 | 0.17 / 0.50 / 0.59 | 0.27 / 0.61 / 0.70 | 0.12 / 0.14 / 0.21 | 3.8 / 5.8 / 6.5 | local_costmap in 9 of 9; one trial also drive_supervision, lidar, localization, odom |
| SIGSTOP controller_server 5 s | M8 rule | non-composed | 10 | 10 / 10 | 0.33 / 0.43 / 0.49 | 0.43 / 0.54 / 0.60 | 0.09 / 0.11 / 0.13 | 3.6 / 5.2 / 5.8 | command_chain + local_costmap; 10 of 10 resumed |
| SIGSTOP controller_server 5 s | control (M7 set) | non-composed | 10 | 10 / 10 | 0.41 / 0.58 / 0.64 | 0.51 / 0.68 / 0.75 | 0.11 / 0.14 / 0.15 | 4.6 / 6.4 / 9.6 | local_costmap |
| SIGKILL / SIGSTOP controller_server | M7 (5 + 5) | non-composed | 10 | 10 / 10 | 0.4-0.64 | 0.60-0.74 | 0.13-0.15 | 5.5-8.5 | local_costmap |

Fault-row host load (median / max of the 1-min average, per row, printed by the tool): rule 6.8-9.8 / 15.2-17.4 (rows incl. the dock rows), control 7.5-9.8 / 17.7-18.9.

Reading the table (every statement is from the cited records):
- **Behavior_server (spin).** With the rule the stop comes at 0.53-0.64 s instead of 1.26-1.34 s and the robot turns about 32 degrees after the signal instead of 64. Within the hold the loop stopped the robot in 24 of 24 trials; the control stopped 0 of 10 SIGKILLs
  (the SIGSTOP control only through a different, late path, next points). The aggregator names `command_chain`; **not in every trial "and nothing else"**: rule SIGKILL rep 4 (`stack_20261115T103738_recovery_probe_chain_faults_007.log`, instance 1) has a second
  stop at 7.89 s whose aggregator fault names `command_chain, drive_supervision, odom` and then also `lidar, localization` (stale in the same 0.35 s: a stall of the watcher or its inputs, not a rule decision; host load 9-11 in that trial). The
  floor is not zero: the rule waits `in_max_age_s` (0.35 s) after the last input, the component's roster `max_age_sec` is 0.15 s and the stand-in has its 0.15 s; the aggregator's first fault is 0.43-0.54 s after the signal and the zero at the gate follows about 0.1 s later.
  A tighter `in_max_age_s` would false-trip the controller (5 Hz, longest period 0.201 s), because the rule cannot tell which commander wrote the last message.
- **What the rule does and does not do.** It flags a short pulse, not a dead commander for good: in all 12 SIGKILL trials (and all 12 SIGSTOP trials) the stand-in is back in RUN 1.6-1.7 s after the signal (`standin_run_from_signal_s` 1.605-1.736 s over the 24)
  while behavior_server is still dead or frozen, because once the smoother's output has gone to zero the stage looks idle again. The robot is stopped; the dead commander is not held down by this rule. Nav2's own lifecycle manager is what then shuts the stack down (next point).
- **What the late `local_costmap` stop is** (M7: "cause not determined"). In all 44 behavior_server trials' stack logs (24 rule + 20 control; SIGKILL and SIGSTOP) Nav2's `lifecycle_manager_navigation` logs "CRITICAL FAILURE: SERVER behavior_server IS DOWN after not receiving a
  heartbeat for 4000 ms. Shutting down related nodes." 3.07-4.07 s after the signal (grep over `m8/{1,2}/stack_*` of the trials) and starts deactivating the managed nodes (for rep 1 of the control, `m8/1/stack_20261115T111034_recovery_probe_faults_002.log`: `collision_monitor`
  at +3.62 s, `velocity_smoother` +3.73 s, `controller_server` and its local costmap +17.88 s). In all 22 SIGSTOP trials (12 rule, 10 control) the lifecycle manager deactivates `controller_server` 17.38-18.68 s after the signal, after the frozen behavior_server finishes
  deactivating, and the second stand-in soft stop follows 18.07-18.94 s after the signal in the 12 rule trials (aggregator first fault 17.97-19.26 s in the control). In the SIGKILL trials the "IS DOWN" line is also present but no `controller_server` deactivation
  appears. This is the Nav2 stack shutting itself down 13+ s after the robot has stopped; it does not explain M7's 8.4 s timing, which has a different workload, and I do not claim it does.
- **Controller_server.** The local costmap (hosted in that process) already stopped both faults at 0.6-0.7 s in M7 and in the control. The rule adds an earlier, second reason (`command_chain` is named in 22 of 22 trials) and
  moves the median stop from 0.61 to 0.52 s (kill) and 0.68 to 0.54 s (stop), the distance from 0.14 to 0.11 m. A small gain, because another watch already covered this fault.
- Not tried with this rule: a SIGKILL of the smoother itself, a backup fault, a commander that bypasses the smoother (docking_server, the reactive coverage planner), and any fault in composed mode.

## Gap B: dock detector (dock_drive in BACK, 10 + 10 exposed; dock stack, no Nav2, composition not applicable)
| Fault | Series | Stopped any time / within hold | Aggregator first fault (s) | Zero at the gate (s, min/med/max) | Reversed after the signal (m) | Rotation (deg) | Blamed |
|---|---|---|---|---|---|---|---|
| SIGKILL dock_detector | M8 rule | 10 / 10 | 1.42 / 1.59 / 2.00 | 1.52 / 1.70 / 2.11 | 0.09 / 0.10 / 0.12 | 0.1 / 0.2 / 0.4 | dock_fix |
| SIGSTOP dock_detector 5 s | M8 rule | 10 / 10 | 1.40 / 1.49 / 1.99 | 1.50 / 1.59 / 2.10 | 0.09 / 0.10 / 0.13 | 0.1 / 0.2 / 1.0 | dock_fix; 10 of 10 resumed |
| SIGKILL dock_detector | M7 (5) | 0 / 0 | - | 2.61 / 2.68 / 3.00 | 0.16 / 0.16 / 0.18 | 0.1 / 0.3 / 0.7 | none |
| SIGSTOP dock_detector 5 s | M7 (5) | 0 / 0 | - | 2.65 / 2.77 / 2.99 | 0.16 / 0.17 / 0.18 | 0.3 / 0.3 / 0.4 | none |

The first fault at 1.40-2.00 s is the bound (1.85 s) minus the age of the last fix at the signal (0 to 0.6 s) plus the aggregator step. dock_drive's own timeout (3.0 s) is what ended the M7 trials. So the rule removes about a
third to a half of the blind reverse (median 0.10 m instead of 0.16-0.17 m) and turns "never detected" into a named fault, but it cannot act faster than the detector's own irregularity allows: the fix gaps inside reversing episodes
are 0.60 s (p50) with 1.2 s gaps in 3.8% and a 2.4 s gap once (M8_RECON section 2).

## Soaks with the rules enabled (`summarize.py --by-task results/fullstack/soak_m8_*_2026-11-15_kitwip.jsonl`)
| Task | Composition | Profile | Sim h | Active h | Work done | Distance | Soft stops | Heartbeat gaps > 0.2 s |
|---|---|---|---|---|---|---|---|---|
| recovery_chain (wall; rule A) | composed | default | 2.0 | 2.0 | 155 goals succeeded, 79 aborted, 4 cut; BT started 217 spins, 165 waits, 93 backups | 0.73 km | **0** | 0 |
| nav_amcl_chain (mix; rule A) | composed | default | 2.0 | 2.0 | 347 NavigateToPose succeeded, 86 cancelled mid-drive, 43 early, 44 unplannable, 44 direct probes; 126 spins / 82 backups / 86 waits started | 0.97 km | **0** | 0 |
| dock_fix (rule B) | no Nav2 | default | 1.0 | 1.0 | 95 docked, 6 timeouts, 2 cut | 0.07 km | 7 (all `drive_supervision`, all 0.22-0.25 s after a harness `set_pose`) | 7 |
| dock_fix (rule B) | no Nav2 | shm_hc60 | 1.0 | 1.0 | 107 docked, 4 timeouts, 2 cut | 0.08 km | 21 (19 `drive_supervision` after a `set_pose`, **2 `dock_fix`**) | 21 |

Rule A: 0 soft stops in 4.0 sim hours of a controller, spin, backup and wait mix (all 4.0 h active: goal loops and recovery scenarios) in composed Nav2, i.e. fewer than 0.75 per hour at 95% (3/T); the M7 default nav_amcl soak had 1 unexplained stop in
2.0 h, the recovery soak 0. The stand-in's counter delta equals the recorder's count. The soaks never exercised a node fault.

Rule B false trips (`m8_stops.py results/fullstack/soak_m8_dock_fix_2026-11-15_kitwip.jsonl`): **2 stops of `dock_fix` in 216 attempts (0.9%; exact 95% interval 0.11-3.3%), both in shm_hc60 (2 of 113; interval 0.2-6.3%), 0 of 103 in the default profile
(interval 0-3.5%); the replay of the recon traces predicted 3 of 89 (3.4%).** Details: shm_hc60 session 1 at 13:41:10.99 and session 2 at 14:26:13.64, 13.6 s and 13.3 s into attempts of 16.4 s and 14.8 s (the end of the approach), robot at 0.06 m/s,
command fresh; both attempts then succeeded. The watcher's warning ("newest dock fix ... 1.86s old > 1.85s") is logged at the FIRST failing 50 Hz check, so it always reads about 1.86 s and says nothing about how far over the bound the
fix age got: the aggregator episodes lasted 1.3 s and 0.7 s (`non_emitting_episode` records), i.e. the fixes were silent for about 2.4-3 s, the end-of-approach dropout seen in the recon, three or more missing fits, not a near miss.
**A third `dock_fix` not-ok did not cause a stop and is not in the stop count:** `m8/2/stack_20261115T133948_dock_fix_soak_002.log`, watcher warning at epoch 1794749727.30 (14:35:27), no aggregator episode recorded (the heartbeat was withheld for less
than the 0.15 s roster limit). So the rule was not-ok three times in 216 attempts, the same 3 as the recon replay predicts per 89. **No threshold was changed after these stops.**
The 26 `drive_supervision` stops after a harness teleport are the M7 pattern but not purely a `set_pose` artefact: each followed a `set_pose` by 0.22-0.25 s and was blamed only on `drive_supervision`, but the shm_hc60 ones cluster: in shm_hc60 session 1, 7 of
its 14 stops fall in minutes 24.0-27.7 at host load 7.6-11.3 (session median 4.3), while the default instance running at the same time had none in that window; the profile order also flipped against M7 (M7 default 11 / shm_hc60 6, here 7 / 19). I describe this as
load- and profile-sensitive and not explained.

## Item D: reactive planner command period, one instance running
One instance running (the other container idle; the unrelated LLM/tunnel containers of the host were running), task `coverage_reactive` with the shipped watch config, 4 sessions of 700 sim s (two on 2026-11-15, two on
2026-11-16; all valid, RTF 0.9998; host load median 2.2 of 12, max 9.9 at a stack start). Traces: `results/fullstack/m8_recon/trace_reactive_{a,b}.jsonl`; sessions: `soak_m8_coverage_reactive_2026-11-15_kitwip.jsonl`;
table: `m8_analysis.py reactive results/fullstack/m8_recon/trace_reactive_?.jsonl`. `/cmd_vel` has several writers and the message info carries no publisher id, so a message counts as the planner's when no `/cmd_vel_smoothed` message came
in the 50 ms before it (the collision monitor republishes within a few ms).

| Planner command period (consecutive planner messages < 1.5 s apart) | n | min | p50 | p99 | max | share of periods > 0.30 s |
|---|---|---|---|---|---|---|
| all | 3597 (12.9 min of planner activity; the sessions' active time is 0.58 h including Nav2 transits) | 0.198 s | 0.200 s | 0.401 s | 0.601 s | **7.2%** (6.8% and 7.6% in the two trace files) |

Headline: **a missed 5 Hz tick (a 0.4 s period) in about 7% of the periods, which also occurs at low host load.** (p95 sits on the boundary between 0.2 and 0.4 s and is not a stable figure; the first M7 and this table differ
there for that reason.) In the first trace file all 122 periods of about 0.4 s have no `/cmd_vel` and no `/cmd_vel_smoothed` message inside them, so they are not an artefact of the attribution. There is no loaded comparison made with this method, so
I cannot say that load makes it worse or better. With the shipped `cmd_max_age_s` 0.15 s the same sessions have 296 soft stops in 0.58 active hours (511 per active hour, 295 blaming only `drive_supervision`, 1 also `odom`); the M7 proposal 0.30 s is
exceeded by about 7% of the periods. Not tuned, as instructed; nothing in the loop was changed for the reactive planner.

## What could NOT be closed from the watcher side, and why
- **A, the last 0.5-0.6 s.** The rule must wait one controller period plus a margin (0.35 s) because both commanders share one input topic and the controller is the slow one (5 Hz); then the aggregator (0.15 s) and the stand-in (0.15 s). The residual is
  about 29-37 degrees of spin, or about 0.1 m of driving at 0.2 m/s for the controller. Closing the rest needs the commander to say it is alive (a heartbeat or a state topic), i.e. a change upstream.
- **A is limited to what goes through the smoother** and flags stale motion, not a dead commander. A dead `docking_server` or reactive planner is seen only by `drive_supervision` (it writes `/cmd_vel` directly, 20 Hz / 5 Hz commands).
- **B keeps a 1.5-2.1 s blind reverse** and a false-trip cost of about 1% of attempts (2 of 216, interval 0.1-3.3%). The detector's fits are irregular at the end of the approach (silences of 2.4-3 s); a bound that covers
  every observed age (2.5 s) would be nearly as long as dock_drive's own 3.0 s, so it would buy almost nothing. B must not be used where something else reverses on the same stream.
- **C stays open on purpose**: with `executor:=nav2` the planner only sends goals; the controller drives and supervision of motion is intact. Whether the coverage job is alive is a task-level question (planner topics, meter progress), not a stop condition.
- **Not tested**: composed-mode faults, a faulted smoother, **backups (never faulted; all 44 behavior trials were spins)**, the `/cmd_vel_nav` rate during direct spin actions (traced only for BT spins and backups), real hardware, any commander other than the ones named.

## Limits
- Single host, at most 2 instances, one sim world per task; the fault trials are 9-12 exposed per cell, so a cell with 12 of 12 bounds the miss rate at about 25% (95%), not at zero.
- The behavior_server trials are all spins from rest at the nominal rate, triggered 0.2 s into the spin; in real BT recoveries a spin can be cut short by "Collision Ahead" within 0.3 s (as in the first attempt), when no fault is possible.
- The bounds come from one 469 s chain trace (5 spin and 2 backup stops) and 89 dock episodes; the 1.5 factor is a margin, not a tail estimate (and the soaks happened on the same sim world as the recon).
- Rule B false trips depend on the detector's end-of-approach behaviour, which differs between worlds and robots.

# M7 (2026-11-14 and 15): AMCL, coverage, docking and behaviour-tree recoveries on the real simulation stack

Raw data: `results/fullstack/soak_m7_<task>_2026-11-14_kitwip.jsonl` (soak sessions), `faults_m7_2026-11-14_kitwip.jsonl` (fault trials),
`results/fullstack/m7/<i>/` (run logs of the three instances; the stack logs there are only those of the last session or trial of each instance),
`results/fullstack/m7_recon/` (graph and rate probes), `results/fullstack/_m7_smoke/` (harness shake-out, not counted). The kit was a dirty
tree on top of `ce0d029` (`kitwip`); aggregator `b2b9796`. Graph facts per stack: `docs/M7_RECON.md`. Every table below is printed by
`python3 bench/fullstack/summarize.py --by-task results/fullstack/soak_m7_*.jsonl` and `summarize_faults.py` (commands in `bench/fullstack/README.md`).
Same setup as M6 (Gazebo, software GL, RTF 1.0, health nodes on wall time, 150 ms soft stop), at most 3 instances in parallel, 1800 sim s per session.

## What was run, what was not
| Task | Stack | Run |
|---|---|---|
| AMCL navigation | `oomwoo_clean nav.launch.py` (AMCL + full Nav2, one composed container); faults with the same call and `use_composition:=False` | yes |
| Coverage | `coverage_planner` + `coverage_meter` on the AMCL stack, `executor` nav2 and reactive | yes |
| Docking | `dock_sim` nodes (ir_beacon_sim, dock_detector, dock_drive) in kitchen_dining, no Nav2 | yes |
| BT recoveries | stock Nav2 BT, a wall the map does not know placed across the path through the gz service | yes |
| `opennav_docking` | the stack runs `docking_server`, but navigation.yaml has only the template (SimpleChargingDock, placeholder poses, no dock model or charge signal in the sim). Not attempted live; no upstream sim support for it | skipped |

Watcher configs: `nav_watch_amcl.yaml` / `nav_rosters_amcl.yaml`, `nav_watch_dock.yaml` / `nav_rosters_dock.yaml`; the M6 cores are
unchanged. Thresholds were not tuned after a run; the one proposed variant (`nav_watch_amcl_reactive.yaml`) is labelled as such below.

## Graph facts that changed a config (details in M7_RECON)
- `/cmd_vel` has 2 writers on the full Nav2 stack (collision_monitor, docking_server), 3 with coverage_planner, 1 (dock_drive) in the dock
  stack. `drive_supervision` counts any writer on the stand-in input by design. Consequence: while one writer is alive, a second dead
  commander is invisible to it (a live writer keeps the stream fresh); the loop supervises the stream, not each writer.
- `/amcl_pose` is silent when the robot is idle; the liveness signal is `map->odom`, 5.0 Hz new stamps moving and stationary (amcl sends
  it from its scan callback). `/tf` has three writers and the tf watch has no writer guard.
- The global costmap footprint is 2 Hz on this stack (5 Hz on the slam stack): roster max_age 1.5 s, advisory.
- `dock_drive` publishes `/cmd_vel` at 20 Hz in every state, zeros included.
- The reactive coverage executor commands `/cmd_vel` at 5 Hz (period 200 ms, p95 201 ms, max 405 ms; Nav2's chain: 20 Hz).
- `/cmd_vel_nav` is 4.97 Hz while `/cmd_vel_smoothed` and `/cmd_vel` are 20.0 Hz (`rates_nav_amcl.json`): the velocity smoother re-emits the
  held setpoint on its own 20 Hz timer, upstream `velocity_timeout: 1.0`, `max_decel` -2.5 / -3.2 (see "Reading the table").

## Soaks: sim time and ACTIVE time
33 sessions of 1800 sim s, all valid, 16.5 sim hours. **Coverage sessions are mostly idle**: the planner log reports `complete` in all 11 coverage
sessions, the meter ends at 0.89-0.92, and the robot does not move after that although the planner process stays alive. Active time
(`summarize.py`, `active_seconds`) is the time up to the last motion evidence in the session record (an uncommanded-motion event, or a health event
with odometry above the thresholds or a command younger than 1 s): 931-1009 s per nav2-executor session, 521-580 s per default-profile and
shm_hc60 reactive session, 437-489 s in the variant sessions. It is an estimate from the events the record keeps, not a direct measurement of
motion; for the goal loops, dock cycles and recovery scenarios the whole window is active. Rates below use active time; "3/T" bounds (95%,
zero events) use it too.

| Task | Profile | Sim h | Active h | Work done | Distance | Soft stops | Per active hour | Heartbeat gaps >0.2 s |
|---|---|---|---|---|---|---|---|---|
| nav_amcl (mix) | default | 2.0 | 2.0 | 356 NavigateToPose succeeded, 88 cancelled mid-drive, 44 early, 44 unplannable, 44 direct probes; 114 spins / 70 backups / 70 waits started | 0.99 km | 1 | 0.5 | 2 |
| nav_amcl (mix) | shm_hc60 | 2.0 | 2.0 | 351 / 87 / 44 / 45 / 45; 120 / 79 / 80 | 0.98 km | 0 | 0 (bound 1.5/h) | 0 |
| coverage, nav2 executor | default | 2.0 | 1.10 | 4 jobs, meter 0.91 each; 468 waypoint goals succeeded | 0.29 km | 0 | 0 (bound 2.7/h) | 1 |
| coverage, nav2 executor | shm_hc60 | 2.0 | 1.07 | 4 jobs, meter 0.89-0.92; 464 waypoint goals | 0.28 km | 0 | 0 (bound 2.8/h) | 0 |
| coverage, reactive (config as shipped) | default | 1.0 | 0.30 | 2 jobs, 0.92 | 0.15 km | 149 (148 active, 1 idle) | **492** | 150 |
| coverage, reactive (config as shipped) | shm_hc60 | 0.5 | 0.16 | 1 job, 0.92 | 0.07 km | 75 | **466** | 75 |
| coverage, reactive, proposal config (cmd 0.30 s) | default | 0.5 | 0.14 | 1 job, 0.92 | 0.07 km | 8 | 59 | 8 |
| coverage, reactive, proposal config | shm_hc60 | 0.5 | 0.12 | 1 job, 0.91 | 0.07 km | 11 | 91 | 11 |
| dock | default | 1.0 | 1.0 | 111 docked, 5 timeouts, 2 cut | 0.09 km | 11 | 11 (all after a harness set_pose) | 11 |
| dock | shm_hc60 | 1.0 | 1.0 | 98 docked, 4 timeouts, 2 cut | 0.08 km | 6 | 6 (all after a harness set_pose) | 6 |
| recovery (wall) | default | 2.0 | 2.0 | 126 goals succeeded, 98 aborted; BT started 250 spins, 208 waits, 110 backups | 0.64 km | 0 | 0 (bound 1.5/h) | 0 |
| recovery (wall) | shm_hc60 | 2.0 | 2.0 | 111 succeeded, 121 aborted; 303 / 260 / 135 | 0.55 km | 0 | 0 (bound 1.5/h) | 0 |

The stand-in's own soft-stop counter delta equals the recorder's count in every row. RTF in 30 s windows: 0.977-1.0.

### Soft stops, classified from the logs
- **Reactive coverage, 224 stops: 223 blame `drive_supervision` only, 1 also `odom` (idle, see below).** A rule/configuration mismatch for this
  commander. The planner's command period is 200 ms and `cmd_max_age_s` is 0.15 s, so `drive_supervision` withholds the heartbeat between commands
  while the robot moves. At the start of the episodes the command was 0.27-0.34 s old (median 0.29); the robot was at 0.3 m/s or more in 128 of
  the 224 (the others were slower or turning). Fail-safe direction, but the shipped configuration cannot run this job: 466-492 stops per active
  hour. Proposal (not a tuned default): `cmd_max_age_s` 0.30. Measured with it: 8 stops in 0.14 active h (default) and 11 in 0.12 active h
  (shm_hc60), i.e. 59 and 91 per active hour, with command age 0.42-0.48 s at the start of the episodes. The variant therefore does not remove the
  problem, and a window covering those gaps (0.5 s) would allow 0.18 m of travel at 0.35 m/s after a dead commander.
  **These three variant/shm sessions ran at host busy 9.4-10.3 of 12 cores (the other 30 sessions: 7.2-8.8) and the lowest RTF (0.977-0.98)**;
  the 0.42-0.48 s command gaps ("the planner pauses about 0.45 s") and the proposal's remaining stops may be load, not planner behaviour.
  The separate probe's 405 ms maximum was taken at normal load.
- **Dock, 17 stops in 222 attempts, all after a successful dock**: each came 0.22-0.25 s after the harness `set_pose` of the next attempt (17 of
  17 within 0.5 s of a recorded set_pose; 0 stops during a manoeuvre in any attempt). Only `drive_supervision` was blamed (never `odom`), the
  aggregator kept publishing every 50 ms, the command was fresh (0.002-0.05 s) and odometry 0. That is consistent with the teleport's odometry
  transient failing `settle_s` while the setpoint is zero, i.e. the rule working as designed; the earlier "stream stall" explanation is not
  supported. The gz CLI is also not isolated as a cause: the same 340 ms stationary /odom maximum appears in `rates_coverage_reactive.json` with no
  teleports. The persistent-binding harness reduced the rate from 9 stops in 12 attempts (gz CLI, first smoke run) to 17 in 222.
- **nav_amcl default, 1 stop** (session 1, 118 sim s, 19:55:45, en route at 0.08 m/s): the aggregator published no stack message for about
  350 ms (messages 0.40 and 0.35 s old, then a fault message naming `odom` and `drive_supervision` stale; a genuinely stale `/odom` would have
  produced fault messages every 50 ms, as in the dock stops). Two recorder heartbeat gaps (350 and 300 ms); stand-in back in RUN 0.40 s after the
  stop. Classification: a stall in the health path (aggregator or its delivery), cause unexplained. It happened 2 min after all three instances were
  launched and no neighbour was starting a stack at that moment. The session-end load average (15.8) was taken 28 min later and says nothing about this event.
- **Neighbour launches**: the nav2-coverage default session's only heartbeat gap (375 ms at 20:54:45.9, stand-in delta 0, so a recorder-side
  stall) came 18-23 s after two neighbour instances started their sessions; the single idle reactive stop (blaming `odom` and `drive_supervision`,
  command 1231 s old, robot still) came 3.2 s after two neighbours started and ended sessions. Two events, not a proof; `tools/sim.sh run` now has
  `OHK_RUN_STAGGER_S`, and `run.py` records the host load as a time series (`loadavg_series`, also in fault records). Not yet exercised in a run.
- Coverage (nav2 executor), recovery and nav_amcl shm_hc60: 0 stops.
- `uncommanded_motion` (odometry above the thresholds with no command newer than 0.15 s): 62 and 84 events in the nav2-executor coverage soaks,
  all with a non-zero held setpoint and at most 0.06 s long: the Nav2 command stream skips a 20 Hz slot now and then (command age at the start
  0.157-0.17 s). No stop followed; the margin to the 0.15 s limit is under 20 ms.

### Recoveries and docking: what the watchers did
- 8 sim hours with BT-triggered recoveries (nav_amcl + recovery soaks: 787 spins, 618 waits, 394 backups started, 89 of the spin/wait/backup
  triples being direct probes): 0 stops in the recovery soaks and 1 unrelated stall in nav_amcl, none during a recovery interval (the nav_amcl stop was in the 'en route' phase). Spin, wait and
  backup commands pass through the smoother and collision monitor at 20 Hz, so `drive_supervision` sees a normal stream.
- Task-level wedges: in all 8 recovery sessions between 2 and 32 aborted goals (travel under 0.5 m) occurred (per session: 10, 20, 12, 27, 31, 2, 32,
  24 aborted goals under 0.5 m); the longest runs of consecutive aborts were 11, 15, 11, 27, 38, 3, 19, 29. One shm_hc60 session (the 38-run) never
  recovered: goals 24-60 all aborted (travel 0.002-1.44 m), the rest of the session. During the run the controller log showed "collision ahead"
  messages (not retained: the stack logs of the soak sessions were overwritten by later runs). The loop stayed healthy while the robot
  did nothing; that is the known limit.
- Dock: 9 attempts timed out after 150 s: 8 in GOTO (travel 0.003-0.121 m), 1 in BACK after 0.729 m. dock_drive's own log said it was holding
  without a dock fix (not retained). Dock logic, not a health fault.

## Faults (5 trials each unless stated; seconds from the signal; kit `kitwip`, default profile)
Per-node faults need one process per node: AMCL, Nav2 and behavior_server faults used `composition:=False`. Table = `summarize_faults.py
results/fullstack/faults_m7_2026-11-14_kitwip.jsonl --latest-label-per-task`. "Exposed" is defined after the trials were seen (post hoc): for
behavior_server faults the robot had to be in a spin at its nominal rate (>= 0.9 rad/s) or a backup (>= 0.10 m/s) at the signal; all other faults
count as exposed (triggered on motion). "Detected within hold" = stand-in soft stop within the 5 s hold.

| Fault | During | Exposed | Detected (any time / within hold) | Zero at the gate (min/med/max s) | Distance to standstill (m) | Blamed |
|---|---|---|---|---|---|---|
| SIGKILL amcl | driving | 5 | 5 / 5 | 0.54 / 0.63 / 0.70 | 0.12-0.15 | localization |
| SIGSTOP amcl 5 s | driving | 5 | 5 / 5 | 0.54 / 0.58 / 0.67 | 0.11-0.14 | localization; 5/5 resumed |
| SIGKILL controller_server | driving | 5 | 5 / 5 | 0.60 / 0.66 / 0.69 | 0.13-0.15 | local_costmap (the costmap lives in that process); once also drive_supervision, lidar, localization, odom |
| SIGSTOP controller_server 5 s | driving | 5 | 5 / 5 | 0.64 / 0.66 / 0.74 | 0.14-0.15 | local_costmap; 5/5 resumed |
| SIGKILL dock_drive | reversing | 5 | 5 / 5 | 0.35 / 0.41 / 0.41 | 0.02-0.03 | drive_supervision |
| SIGSTOP dock_drive 5 s | reversing | 5 | 5 / 5 | 0.32 / 0.38 / 0.41 | 0.02-0.03 | drive_supervision; 5/5 resumed |
| SIGKILL dock_detector | reversing | 5 | **0 / 0** | 2.61 / 2.68 / 3.00 (dock_drive's own 3 s hold) | 0.16-0.18 | none |
| SIGSTOP dock_detector 5 s | reversing | 5 | **0 / 0** | 2.65 / 2.77 / 2.99 | 0.16-0.18 | none |
| SIGKILL coverage_planner, nav2 executor | transit by Nav2 (0.2 m/s) | 5 | **0 / 0** | 6.0-6.2 (one trial 11.2) | 0.49-0.51 | none |
| SIGSTOP coverage_planner 5 s, nav2 executor | transit | 5 | **0 / 0** | 6.0-6.2 (one 11.0) | robot still moving at the end in 5/5 (1.6-2.0 m) | none |
| SIGKILL coverage_planner, reactive executor, proposal config | reactive pass (>= 0.30 m/s) | 5 | 5 / 5 | 0.42 / 0.51 / 0.55 | 0.18-0.22 | drive_supervision |
| SIGSTOP coverage_planner 5 s, same | reactive pass | 5 | 5 / 5 | 0.37 / 0.44 / 0.54 | 0.16-0.22 | drive_supervision; 5/5 resumed |
| SIGKILL behavior_server | spin (4) or backup (1) | 5 | **0 / 0** | 1.01 / 1.29 / 1.32 | 0.00-0.15 | none; the 4 spin trials rotated 60.8-64.4 degrees after the signal (corrected, see M8 "Rotation"), the backup 0.0 |
| SIGSTOP behavior_server 5 s | spin | **2** exposed of 4 valid (the other 2 hit the tail of a spin: 0.71 and 0.32 rad/s at the signal, rotation 5.4 and 0.7 degrees after correction) | 2 / **0** | 1.23 / 1.29 / 1.36 | 0.00 | both exposed trials rotated 58.8 and 66.3 degrees undetected; a soft stop blaming local_costmap followed 3.4-3.6 s AFTER the SIGCONT in all 4 valid trials (8.4-8.6 s after the signal; M8 found Nav2's lifecycle manager shutting the stack down in its own logs, with different timing), cause not determined |

Rotation is the change of the ground-truth yaw from the signal to standstill, interpolated between the samples (about every 0.25 s); **corrected in M8**: M7 integrated the yaw rate only up to the last sample before standstill and under-reported (the values here and in the table are the corrected ones); SIGSTOP rep 1 of the same
series did not arm (local_costmap missing) and is invalid.
Excluded old valid trial (series `m7_faults`, SIGSTOP behavior_server rep 1, trigger dwell 2 s): the robot kept driving on controller commands for
4.47 s and 0.57 m, then stopped at 4.85 s through `drive_supervision` + `local_costmap` (soft stop 4.85 s). It was not in a spin; it shows the
stale-setpoint path working after the BT stalled, not the behavior_server case.

Reading the table:
- **A dead commander behind the velocity smoother is invisible, and `drive_supervision` measures the smoother's timer, not the commander.** The
  controller emits `/cmd_vel_nav` at 5 Hz while `/cmd_vel_smoothed` and `/cmd_vel` stay at 20 Hz: the smoother re-emits the held setpoint until
  its upstream `velocity_timeout` (1.0 s), then decelerates (`max_decel` -3.2 rad/s^2 for a spin at 0.96 rad/s: about 0.3 s), predicting about
  1.3 s. Measured first zero at the gate for the four spin kills: 1.26-1.32 s (standstill 1.28-1.34 s). The same 0.84-0.97 s of non-zero commands
  after the signal is seen when `controller_server` is killed or stopped (the loop still stops at 0.60-0.74 s, but through the local costmap watch,
  not through the commands). The stop belongs to Nav2, not to the health loop, for every commander upstream of the smoother
  (controller_server, behavior_server).
- Composition caveat: this applies to the non-composed launch used for per-node faults. In the default composed launch a crash of one node kills
  the whole `nav2_container` process, which the M6 faults showed the loop does detect (SIGKILL Nav2: 0.35-0.41 s). The behavior_server row is a
  per-node SIGKILL that the upstream launch does not produce by itself.
- The nav2-executor coverage planner commands goals, not motion: after its death the controller finishes the current waypoint (6.0-6.2 s, 0.5 m) and the
  loop stays green; with SIGSTOP the job resumes after 5 s. Correct for a loop that supervises motion, but "coverage server dead" is not a stop
  condition on this architecture.
- The dock detector can die or freeze while the robot is reversing: dock_drive carries its estimate by odometry for 3 s, then holds. The loop never
  notices (the dock fix is silent for seconds legitimately: 1.5 Hz, max gap 4.3 s).
- Killing `controller_server` is caught by the local costmap watch because that process hosts the local costmap.
- Stop times of 0.54-0.74 s for AMCL and controller are the 0.6 s roster max_age of `localization` / `local_costmap` plus aggregator and stand-in. The
  0.15 s path (dock) stops in 0.32-0.41 s; the reactive variant has a 0.30 s command window and stops in 0.37-0.55 s.
- Earlier trial series are kept in the raw file with label `m7_faults`: behavior_server faults with a 2 s trigger dwell (4 of 10 invalid: three did
  not reach the condition in 150 s, one did not arm; spin and backup do not last 2 s) and reactive-coverage faults triggered at any speed >= 0.12 m/s
  (all hit Nav2 transits at 0.2 m/s, not the planner's own passes, so the `_fast` trigger was added). `summarize_faults.py --label=m7_faults --task=...`
  prints them.
- Fault records carry no host load. The v2 behavior and reactive fault series started right after the three overloaded sessions (02:10); their
  stop times could include load effects. Not checked.

## Limits
- Single host, three parallel instances; host busy 7.2-8.8 of 12 cores in 30 sessions and 9.4-10.3 in three; RTF 0.977-1.0.
- Active time: 2.0 h per task and profile for nav_amcl and recovery, 1.1 h for coverage (nav2 executor), 1.0 h for dock, 0.12-0.30 h for the
  reactive rows. A row with zero stops bounds the rate only to about 3/T at 95% (1.5/h for 2 h, 2.7/h for 1.1 h).
- The two Fast DDS profiles do not differ here: nav_amcl 1 vs 0 stops (one unexplained event), dock 11 vs 6 (both harness-related), reactive
  492 vs 466 per active hour.
- Docking is dock_drive with a simulated IR beacon; the fit uses the LiDAR, the robot is teleported between attempts, there is no charging.
- Coverage jobs finish the sweep (planner `complete`, meter at the 0.90 target) in about 520-1010 s; the planner then idles. Completion times of the
  full job were not measured separately.
- Recovery provocation is one geometry (a 2.4 m wall 0.7 m ahead on the straight line to the goal), persistent or removed after 8 s.
- Fault trials with composition false: the Nav2 processes are the same nodes but not the upstream default launch.
- The watcher proves output exists and is fresh; the dock-detector, behavior-server and coverage-planner rows show what it cannot see.

# M6 (2026-11-13 and 14): the health loop on the real simulation stack

Raw data: `results/fullstack/` (`soak_long_2026-11-13_kit8d7211e.jsonl`, `soak_long2_2026-11-14_kit35f0bba.jsonl`,
`_m6c_fix2_faults`, `_m6c_fix2_item6`, earlier passes in the other `_*` directories). How to run: `bench/fullstack/README.md`.
Design and limits: `docs/SPEC.md`, sections M6a and M6c. Everything above this section used fake components; this
section is the first with real nodes.

Setup: Gazebo (living_room world, lidar and odometry, software rendering) + Nav2 + slam_toolbox localization from the
`makerspet/oomwoo:jazzy-dev` image, real-time factor 1.0, one x86 PC, up to four instances in parallel containers.
The aggregator (`b2b9796`) is fed by the kit's topic watcher, which turns activity on real output topics into
component heartbeats, with a static roster. The MCU stand-in (150 ms soft stop) gates `cmd_vel` on the hop before
`ros_gz_bridge`. Health nodes run on wall time, so header-stamp checks are off.

## Rates measured on the running stack, and the roster used
| Component | Signal watched | Writers | Rate driving | max_age_sec | Class |
|---|---|---|---|---|---|
| lidar | `/scan` | 1 | 5.0 Hz | 0.6 | critical |
| odom | `/odom` | 1 | 50 Hz | 0.15 | critical |
| localization | new `map`->`odom` stamps on `/tf` | not guarded | 5.0 Hz | 0.6 | critical |
| drive_supervision | `/cmd_vel` (stand-in input) + `/odom` | 1 | 20 Hz | 0.15 | critical |
| local_costmap | published footprint | 1 | 5 Hz | 0.6 | critical |
| global_costmap | published footprint | 1 | 5 Hz | 0.6 | advisory |

- Only the drive path fits the 20 Hz / 0.15 s draft values; the 5 Hz sources need about 0.6 s.
- slam_toolbox `/pose` is 1.4 Hz moving and silent when stationary, so it cannot serve as a liveness signal.
- `drive_supervision` beats only on a fresh non-zero command, or when the last command was zero and odometry has
  settled (or a short coast window is still open). A non-zero held setpoint without a fresh command never beats.
  This supervises motion whatever node commands it, without a job-following roster. A job-following roster was
  also built and measured against fake Nav2-like timing: it stopped the robot twice per goal (0.6-0.8 s each),
  because the aggregator restarts arming on a roster change; it stays optional.

## Navigation without faults
- Goal accounting on the fixed code (4 sessions of 700 sim s): 228 NavigateToPose goals (148 succeeded, 36 cancelled
  mid-drive, 20 cancelled early, 4 cut by session end, 20 unplannable with no motion) plus 20 direct spin / backup /
  wait actions: 0 stand-in soft stops and 0 periods in which the aggregator withheld the heartbeat, in every phase.
  An earlier pass on the previous rule gave the same result over 220 goals, 20 unplannable goals and 20 probes.
- Soak 1 (kit `8d7211e`), 40 valid one-hour sessions, 20 sim hours per profile:

| Fast DDS profile | Goals | Distance | Soft stops | Heartbeat gaps > 0.2 s |
|---|---|---|---|---|
| default | 6276 | 10.8 km | 1 | 1 |
| shm_hc60 | 6279 | 10.8 km | 0 | 0 |

  The soft stop (and one more in a session cut short when the soak was stopped) came 1-2 s into a session's first
  goal: the aggregator reported `drive_supervision` stale for one or two ticks (and `odom` once), the recorder saw a
  400 ms heartbeat gap, and the stand-in was back in RUN after about 0.35 s. Host load average was about 19 on 12
  threads. Cause not determined; both were on the same instance.
- Soak 2 (kit `35f0bba`, profiles swapped between instances), 14 valid one-hour sessions per profile:

| Fast DDS profile | Goals succeeded / cancelled / aborted | Distance | Soft stops | Withheld-heartbeat periods | Gaps > 0.2 s |
|---|---|---|---|---|---|
| default | 3048 / 1024 / 336 | 7.56 km | 0 | 0 | 0 |
| shm_hc60 | 3045 / 1024 / 336 | 7.56 km | 0 | 0 | 0 |

  The stand-in's own soft-stop counter agrees (delta 0 in every session). One 20 ms interval was recorded in which
  the robot turned on a non-zero setpoint whose last command was 156 ms old, just past the 150 ms window, without
  a stop following; host load average reached 23, so the margins here are thin under this load.
- Together: 68 sim hours, about 21,300 goals, about 36.7 km, 1 soft stop in valid sessions. That is not "no false
  stops": the one that occurred is unexplained, and four instances overload this PC (RTF stayed at 0.997-0.998).

## Faults injected into real processes (kit `8d7211e` plus the fixes, 5 trials each, robot driving, seconds from the signal)
| Fault | Zero command at the gate | Standstill | Distance | Components blamed | Recovers by itself |
|---|---|---|---|---|---|
| SIGKILL aggregator | 0.11-0.15 | 0.23-0.26 | 0.03-0.04 m | none (stand-in timeout) | no |
| SIGKILL topic watcher | 0.25-0.29 | 0.37-0.40 | 0.06 m | all | no |
| SIGSTOP Nav2 for 5 s | 0.37-0.41 | 0.48-0.51 | 0.08-0.09 m | drive_supervision, local_costmap | 4 of 5 within 7.7-8.3 s of SIGCONT |
| SIGKILL Nav2 | 0.35-0.41 | 0.46-0.53 | 0.08-0.09 m | drive_supervision, local_costmap | no |
| SIGKILL slam_toolbox | 0.51-0.71 | 0.61-0.83 | 0.11-0.15 m | localization, local_costmap | no |
| Gazebo paused for 5 s | 0.14-0.27 | (physics frozen) | n/a | drive_supervision, lidar, localization, odom | yes, 0.8-9 s after unpause |
| SIGKILL ros_gz_bridge | 0.24-0.28 | see below | see below | drive_supervision, lidar, localization, odom | no |

- Bridge kill is a simulation artefact, not a stopping result. The gate sits before `ros_gz_bridge`, the only path to
  the simulated motors: the loop detected the kill in 0.24-0.28 s but could not deliver the zero, and the simulated
  base kept its last command for 12.6-15.0 s and 1.7-2.1 m, ending 0.12 m from a wall in 5 of 5 trials (contact
  inferred from pose and map, not recorded). On the robot the MCU is the last hop; loss of the link to the MCU was
  not tested.
- One SIGSTOP trial did not recover within 25 s; cause not determined.
- Nav2 runs as one composed process, so these are process faults, not per-node faults.
- After a writer is SIGKILLed and respawned, the dead DDS participant still counts as a second writer for its lease:
  the watcher's single-writer guard then withholds the heartbeat for about 20 s (one run). Fail-safe, but a long stop.

## What these results do not show
- Nothing about hardware, AMCL, docking, cleaning jobs, behaviour-tree-triggered recoveries, RTF other than 1.0,
  health nodes on sim time, or header-stamp checks.
- A watcher proves that a node produces output, not that the output is right. The local-costmap watch stayed alive
  with `/scan` dead: it shows the update loop runs, not that obstacle data is fresh. `/tf` watches have no writer guard.
- `drive_supervision` does not catch motion that contradicts a live command, motion below its thresholds on a zero
  setpoint, or odometry that is alive and wrong while commands are fresh. It relies on the MCU never restoring an
  old setpoint after a soft stop.
- The 0.5 s coast window is justified only for this simulation at 0.2 m/s (measured coast 0.11-0.12 s).
- Five trials per fault and single runs elsewhere; no confidence intervals.

> Note, 2026-11-10: the M3 measurements further down were taken with the earlier stand-in settings (0.2 s soft stop,
> 2.0 s CPU reset). The kit now uses 0.15 s / 300 s, following the firmware bring-up policy
> (makerspet/oomwoo-firmware#1); those are experimental simulation settings, not a recommendation. The M3 numbers are
> unchanged, but the section directly below supersedes two of their conclusions.

# Addendum 2026-11-10: soak with the 150 ms soft stop, and a transport stall after a node joins

Raw data: `results/soak_pass1_softstop150.jsonl` (soak), `results/diag/` (diagnosis), `results/diag/review/` (independent
re-check). Scripts: `bench/soak.py`, `bench/diag/`. Aggregator `b2b9796`, Fast DDS 2.14.7 with its default SHM + UDP
transports, one PC, container with 1 GiB `/dev/shm`.

## Soak, pass 1
18 cells: roster max_age 0.10 / 0.15 / 0.25 s x 20 / 50 Hz x idle / 12 busy loops / mixed (10 busy loops, one memory
rewriter, one throttled disk writer; not memory pressure). Per cell: 30 s warm-up, a 10-minute steady window, then 40
SIGKILL/restart cycles. All 18 cells were valid.

- Steady windows: 0 false trips in all 18 (3 hours in total). Zero events in T hours only bounds the rate below about 3/T
  per hour at 95%, so below about one per hour here.
- Kill to zero velocity, median per cell: 192-221 ms at max_age 0.10 s, 244-281 ms at 0.15 s, 348-385 ms at 0.25 s.
- Restart cycles: 103 soft stops not caused by a kill, in 720 cycles (14%), by the stand-in's cumulative counter; 95 of
  them have a detail record, and those came a median 0.34 s after the stand-in re-armed. By load: idle 25, cpu 36,
  mixed 42. By max_age: 31 / 36 / 36. They occur idle and at every max_age, so neither is the cause, but they are more
  frequent under load (see the wide grid below). Corrected 2026-11-12: this line first said 95 and "spread evenly",
  counting only the stops with a detail record.
  A few kills landed during such a stop and show latencies of 10-140 ms; those are not real detect-to-stop times.

## Cause of the restart-tail stops
Each traced case was a 333 ms sleep in the Fast DDS shared-memory port health check (`healthy_check_timeout_ms`, default
1000, divided by 3 per failed check; verified in the 2.14.7 source), taken when one node first sends to another node's
port. It runs in a receive/event thread and holds the participant's send lock, so that node's publishes stall for about
0.33 s and then arrive in a burst. In 11 of 11 traced flap cycles such a sleep overlapped the gap: 7 in the aggregator,
4 in the restarted component. The aggregator and the stand-in reacted correctly to a real delivery gap.

| Condition (idle, max_age 0.15 s, 20 Hz) | Soft stops after re-arm |
|---|---|
| soft stop 0.15 s, restart of a critical node | 28 of 200 cycles |
| soft stop 0.20 s | 23 of 100 |
| soft stop 0.30 s | 22 of 80 |
| unrelated silent node started, then stopped cleanly (SIGINT), no critical node touched | 9 of 70 |
| same node, SIGKILLed | 2 of 70 |
| UDP only (SHM transport disabled) | 2 of 100 |
| SHM kept, `healthy_check_timeout_ms` = 60 | 0 of 80 |

- A longer deadline did not reduce it: the stall is longer than every deadline tried.
- It is triggered by a node joining, not by SIGKILL: the stalls began 2.2-2.45 s after the unrelated node started, before
  it was stopped.
- Rates varied between runs from 6% to 28%; no rate here is portable.
- Caveat on the last two table rows: those runs used `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, under which the rmw adds
  its own SHM transport whatever the XML profile says, so "UDP only" still had SHM and the 60 ms setting did not apply to
  every transport. They are superseded by the transport comparison below.
- Not claimed: a Fast DDS bug, that all 103 soak stops were traced, or that disabling SHM is free (large messages lose
  zero-copy; cost not measured; 2 of 100 remained, untraced). `healthy_check_timeout_ms` = 60 was tested idle only; under
  load it may wrongly declare ports dead. Untested: load, ARM, Fast DDS 3.x, other RMWs, the micro-ROS agent.

Consequence: with this middleware configuration, any heartbeat deadline under about 0.4 s is sensitive to nodes joining
the graph, so a restarted node can cause a second stop shortly after motion is re-enabled.

## Transport comparison, soak passes 1-4 (2026-11-11)
Raw data: `results/soak_transport_passes1-5.jsonl` (the table uses passes 1-4; pass 5 is included as raw data). max_age 0.15 s, 20 Hz, soft stop 0.15 s; per cell a 10-minute steady
window and 40 SIGKILL/restart cycles; 4 passes, so 160 cycles per profile and load. Profiles: `default` (no XML, as in pass 1
above), `default_subnet` (no XML, discovery range SUBNET: the control for the other two), `shm_hc60` (SHM kept,
`healthy_check_timeout_ms` = 60), `udp_only` (`bench/profiles/`).

| Profile | Discovery range | Extra stops, idle | cpu | mixed | Total | Heartbeat gaps > 0.2 s not after a kill | `fastrtps_*` files in /dev/shm at cell end |
|---|---|---|---|---|---|---|---|
| default | LOCALHOST | 14 / 160 | 29 / 160 | 35 / 160 | 78 / 480 (16.2%) | 112 | 42-42 |
| default_subnet | SUBNET | 9 / 160 | 26 / 160 | 17 / 160 | 52 / 480 (10.8%) | 74 | 42-44 |
| shm_hc60 | SUBNET | 2 / 160 | 1 / 160 | 6 / 160 | 9 / 480 (1.9%) | 12 | 42-44 |
| udp_only | SUBNET | 0 / 160 | 0 / 160 | 0 / 160 | 0 / 480 (0.0%) | 0 | 6-6 |

- Steady windows: 0 false trips in all 48 cells (8 hours in total).
- `shm_hc60` against its control `default_subnet`: 9 against 52 extra stops in 480 cycles each, under load as well as idle.
  It reduces the effect about six-fold and does not remove it. The 9 remaining stops are all in the re-arm tail with the
  aggregator reporting one critical component stale; their cause is not traced.
- `udp_only`: 0 extra stops and 0 gaps in 480 cycles. The script's validity check flags these cells because 6
  `fastrtps_*` files were present in `/dev/shm` (42-44 with SHM in use). Checked on a live cell: the files are mapped only by
  the benchmark's own probe process, which keeps default transports in every cell (Fast DDS reads the profile once per
  process, and the probe process outlives the cells); the aggregator, stand-in, roster publisher and components map none,
  and the count does not grow over 40 restarts. So the path component -> aggregator -> stand-in ran over UDP only, and the
  result holds for that path, with the caveat that the measuring probe was still an SHM participant. The same caveat
  applies to `shm_hc60`: the probe used the default 1000 ms health check there.
- `default` against `default_subnet`: 78 against 52. Run-to-run variation is large (6-28% in earlier runs), so this is
  not evidence that the discovery range matters; both show the stall.
- Median kill-to-zero latency was 220-276 ms in every valid cell, so the 60 ms health check did not slow detection.

## Rerun with the probe on the cell's profile (2026-11-11)
Raw data: `results/soak_transport_rerun_2026-11-11.jsonl`. Same cells as the comparison above, but each cell now runs in a
fresh process, so the measuring probe uses the cell's Fast DDS profile as well. 5 passes, 200 SIGKILL/restart cycles per
profile and load, all 45 cells valid; `udp_only` cells had 0 `fastrtps_*` files in `/dev/shm`, the others 42-44.

| Profile | Extra stops, idle | cpu | mixed | Total | Heartbeat gaps > 0.2 s not after a kill |
|---|---|---|---|---|---|
| default_subnet | 9 / 200 | 25 / 200 | 24 / 200 | 58 / 600 (9.7%) | 83 |
| shm_hc60 | 0 / 200 | 0 / 200 | 0 / 200 | 0 / 600 | 0 |
| udp_only | 0 / 200 | 0 / 200 | 0 / 200 | 0 / 600 | 0 |

- Steady windows: 0 false trips in all 45 cells (7.5 hours). Median kill-to-zero latency 213-277 ms per cell in all three.
- The control reproduces the earlier rate (52 / 480 before, 58 / 600 now).
- `udp_only` is now confirmed with every participant on UDP.
- `shm_hc60` went from 9 / 480 to 0 / 600. The only change was the probe, which earlier still used the default 1000 ms
  health check; that it caused the 9 earlier stops is an inference, not traced.
- Zero in 600 cycles bounds the rate below about 0.5% at 95%; it does not show the effect is gone. Still one x86 PC,
  Python nodes, max_age 0.15 s at 20 Hz only, no large topics, no micro-ROS agent.

## Wide grid for the 60 ms health check (2026-11-12 and 13)
Raw data: `results/soak_widegrid_part1_2026-11-12.jsonl`. Every participant on the cell's profile. max_age 0.10 / 0.15 /
0.25 s x 20 / 50 Hz x idle / cpu / mixed, 40 SIGKILL/restart cycles per cell, 54 cells, all valid.

| Profile | Extra stops | idle | cpu | mixed | Heartbeat gaps > 0.2 s not after a kill |
|---|---|---|---|---|---|
| default_subnet | 138 / 1200 (11.5%) | 24 / 400 (6%) | 47 / 360 (13%) | 67 / 440 (15%) | 182 |
| shm_hc60 | 0 / 960 | 0 / 320 | 0 / 360 | 0 / 280 | 0 |

- `shm_hc60` had no extra stop and no gap in any of the six max_age / rate combinations (120-200 cycles each).
- With default transports the effect appears at every max_age and rate (7-16% per combination) and about twice as
  often under load as idle; the same idle-to-load ratio shows in the rerun above (9, 25, 24 of 200).
- Steady windows: 0 false trips in all 54 cells (9 hours).
- Median kill-to-zero latency per cell with `shm_hc60`: 166-238 ms at max_age 0.10 s, 259-278 ms at 0.15 s, 331-391 ms
  at 0.25 s; with default transports 193-242, 245-287, 316-373 ms.

Second part (2026-11-12/13, `results/soak_widegrid_part2_2026-11-13.jsonl`, 41 cells, all valid, same grid):
`default_subnet` 90 / 880 (10.2%; idle 8 / 240, cpu 45 / 320, mixed 37 / 320; 13-17 extra stops in each of the six
max_age / rate combinations), 141 gaps; `shm_hc60` 0 / 760 with 0 gaps in every combination. Steady windows: 0 false
trips in all 41 cells.

Totals with every participant on the profile under test (rerun plus both wide-grid parts): default transports 286 extra
stops in 2680 restart cycles (10.7%), `shm_hc60` 0 in 2320, `udp_only` 0 in 600. Zero in 2320 bounds the rate below about
0.13% at 95%. Steady state over the same runs: 0 false trips in 140 ten-minute windows (23 hours).

## What this supersedes below
- "55 kill/restart cycles, none unexplained" (M3, Part A): superseded. That pass saw zero; the difference is not explained
  (untested differences: 64 MiB `/dev/shm` never cleaned, no warm-up, one DDS domain for all cells).
- The two 43 / 47 ms kills in the first M3 kill pass fit this stall.
- The 358-383 ms loaded heartbeat gaps in the M3 table were attributed to probe delay; they match this stall instead
  (not verified).
- Kill latencies, the slow-sim-clock result and the stopping distance stand, but a stop budget should allow about
  0.33 s more for a stall.

# M3 findings: load, jitter, sim clock, stopping distance

All numbers come from `results/*.json`; anything not listed was not measured. One PC (6 cores / 12 threads), container
`ohk-dev`, ROS 2 Jazzy, aggregator pinned at `b2b9796`.

## Reproduce
    tools/dev.sh build && tools/dev.sh test    # M2: 61 tests; appends to /tmp/ohk_results.json in the container
    B=/ws/src/oomwoo_health_kit/bench
    tools/dev.sh exec "python3 $B/part_a.py sweep"    # results/part_a_trips.json (~25 min)
    tools/dev.sh exec "python3 $B/part_a.py kills"    # results/part_a_kills.json (~8 min)
    tools/dev.sh exec "source /ros_ws/install/setup.bash && python3 $B/part_b.py all"   # results/part_b.json (~30 min)

Run Part A and B one at a time on an idle container; `bench/` is not part of `colcon test`. Part B runs `ros2 launch oomwoo_gazebo world.launch.py headless:=true robot_model:=oomwoo_one world:=<bench world>` with
lidar, ranges, tof, cameras and imu set to false. `empty.world` declares no Gazebo systems (no physics, no spawn service), so
`bench/worlds/empty_systems.world` adds Physics, UserCommands and SceneBroadcaster (no Sensors system).

## M2 scenarios (results/m2_ohk_results.json, 7 runs each, min / median / max)
Scenarios a, d, e, g, i (no stop expected or never armed) passed in all 7 runs; a: 0 transitions in 5 s.

| Scenario | Metric | Value (ms) |
|---|---|---|
| b kill critical | SIGKILL -> first zero Twist | 285 / 335 / 337 |
| c hang (SIGSTOP) | signal -> zero Twist | 291 / 303 / 342 |
| c hang | re-arm after SIGCONT (20 Hz probe quantisation) | 398 / 450 / 451 |
| f aggregator killed | kill -> zero Twist | 184 / 187 / 194 |
| f aggregator killed | kill -> CPU reset | 1984 / 1987 / 1994 |
| h work loop stalled (SIGUSR1) | signal -> zero Twist | 293 / 305 / 338 |
| j frozen /clock, stand-in clock_source=ros | freeze -> zero Twist | 201 / 235 / 245 |

## Part A: false trips and kill latency, no Gazebo (results/part_a_trips.json, part_a_kills.json)
Loop: aggregator, stand-in (0.2 s soft stop), roster, 2 critical + 1 advisory components, constant cmd_vel probe (20 Hz).
One 60 s run per cell. Loaded = 12 busy-loop processes (`while True: pass`). Trips = transitions out of RUN, taken from the
stand-in's cumulative soft-stop and reset counters. "hb gap" is the gap between aggregate `mcu_heartbeat` messages as received by
the probe. Loaded gaps over 200 ms are probably probe delays, not real gaps (inference): those runs received 1200 of 1200
heartbeats, with 0 soft stops and 0 zero Twists. A 180 s rerun at 0.10 s / 50 Hz loaded showed no gap over 120 ms.

| max_age s | rate Hz | trips idle | trips loaded | max hb gap idle ms | max hb gap loaded ms | kill->stop loaded ms, min / median / max (n=5) |
|---|---|---|---|---|---|---|
| 0.10 | 10 | skipped (period 100 ms >= max_age) | skipped | - | - | - |
| 0.10 | 20 | 0 | 0 | 50.4 | 61.2 | 255 / 259 / 267 |
| 0.10 | 50 | 0 | 0 | 50.8 | 359.7 | 253 / 256 / 285 |
| 0.15 | 10 | 0 | 0 | 50.7 | 67.6 | 242 / 258 / 299 |
| 0.15 | 20 | 0 | 0 | 50.6 | 69.9 | 286 / 290 / 340 |
| 0.15 | 50 | 0 | 0 | 50.6 | 66.9 | 295 / 322 / 349 |
| 0.25 | 10 | 0 | 0 | 50.8 | 59.7 | 316 / 372 / 425 |
| 0.25 | 20 | 0 | 0 | 50.9 | 61.5 | 390 / 396 / 403 |
| 0.25 | 50 | 0 | 0 | 50.6 | 375.2 | 397 / 445 / 445 |
| 0.50 | 10 | 0 | 0 | 50.9 | 60.0 | 576 / 626 / 681 |
| 0.50 | 20 | 0 | 0 | 50.9 | 382.7 | 622 / 670 / 673 |
| 0.50 | 50 | 0 | 0 | 50.5 | 358.6 | 650 / 657 / 684 |

- No configuration produced a false trip in its 60 s steady-state window, idle or loaded. The smallest tested max_age with zero
  loaded trips is 0.10 s (20 and 50 Hz).
- Three loaded runs saw 19.2 Hz heartbeats instead of 20 Hz at the probe (1152-1153 of 1200). Cause not established.
- Unexplained trips outside the steady-state windows: in a first kill pass (part_a_kills_first_pass.json, 0.15 s / 50 Hz) two
  kills showed zero output 43 and 47 ms after the kill, less than one probe period and far below the ~280 ms floor of that cell.
  The stand-in had therefore already left RUN, within about 1 s of a confirmed re-arm, with no fault injected; most likely a trip
  while the restarted component was starting under load. Soft stops were not recorded in that pass. A 20-kill rerun of the same
  loop did not reproduce it. The script now counts soft stops over the whole kill loop, restart phases
  included: the final pass (part_a_kills.json, 55 kills) had 55 soft stops, none unexplained. The table uses the final pass;
  part_a_kills_second_pass.json is the pass before that change. Zero false trips is therefore established for the 60 s
  steady-state windows and for 55 kill/restart cycles, not in general.
- 12 equal-priority busy loops is mild stress (the scheduler favours mostly sleeping ROS processes). No memory, I/O or DDS-traffic
  stress was tried.

## Part B: Gazebo headless (results/part_b.json, part_b_first_pass.json)
Setup: the stand-in sits between the commander and the robot. Commander publishes `cmd_vel_in`, stand-in publishes `cmd_vel`.
`ros2 topic info -v /cmd_vel`: 1 publisher (`mcu_standin`), 1 subscriber (`ros_gz_bridge`); the Gazebo log shows the DiffDrive plugin
subscribed to `cmd_vel`, and `gz topic -i` shows one publisher and one subscriber. All kit nodes use `use_sim_time`. Max_age 0.15 s, 20 Hz.
Real-time factor (RTF) with the five sensors off: 1.0 (idle sample and all runs); with sensors on: not measured. To test a slow
clock, RTF was forced with the `set_physics` service (`real_time_factor`); the measured RTF equalled the target each time.

False trips, 60 s of sim time (RTF 0.1 row: 30 s):

| RTF | clock_source=steady | clock_source=ros | max hb gap wall / sim ms |
|---|---|---|---|
| 1.0 | 0 trips | 0 trips | 52 / 50 |
| 0.5 | 0 trips | 0 trips | 101 / 50 |
| 0.25 | 155 trips (soft stop 4219 zero Twists, ended in SOFT_STOP) | 0 trips | 201 / 50 |
| 0.1 | never armed within 90 s wall (gap 500 ms > 200 ms soft stop) | 0 trips, 300 s wall | 501 / 50 |

One run per RTF/clock pair (part_b.json carries the trip rows over from the first pass). A slow sim clock does false-trip a
stand-in that measures timeouts on a steady clock, once the wall-time heartbeat gap (50 ms / RTF) reaches the soft stop (0.2 s):
RTF 0.5 passed, RTF 0.1 never armed. At RTF 0.25 the gap equals the soft stop exactly, so the 155 trips are jitter at that
boundary, not a rate. With
`clock_source=ros` no trip occurred at any RTF tested, because aggregator and stand-in then share the sim clock.

Stopping distance, straight line, 0.2 m/s, SIGKILL of `critical_a`, 5 runs, RTF 1.0, `clock_source=steady`, empty world:

| Metric | min | median | max |
|---|---|---|---|
| kill -> first zero Twist, ms (wall) | 307 | 311 | 313 |
| distance kill -> standstill, ground-truth `/odom`, m | 0.0717 | 0.0725 | 0.0729 |
| time kill -> standstill, truth, s (sim = wall at RTF 1.0) | 0.391 | 0.400 | 0.403 |
| distance, wheel odometry `/odom_wheel`, m | 0.0616 | 0.0624 | 0.0628 |

Standstill = first sample within 0.5 mm of the final rest pose. About 0.062 m is travelled before the zero Twist (0.2 m/s x 0.311 s)
and about 10 mm after; the split is good to about 4 mm because sim time is sampled from `/clock` every 20 ms. Wheel odometry
stops 0.311-0.323 s after the kill, with the zero Twist: the model's wheels stop at once and the body slides the last 10 mm,
which is why wheel odometry reads 10 mm short. Deceleration is not ramped in this model. An earlier pass (part_b_first_pass.json)
gave 0.070 / 0.072 / 0.079 m. No cleaning speed was found in the oomwoo config; 0.2 m/s is `desired_linear_vel` of the Nav2
controller in `oomwoo_one/config/navigation.yaml`. Other speeds: not measured.

## Recommended timing (derived from the numbers above)
| Parameter | Value | Basis |
|---|---|---|
| Component rate | 20 Hz | zero trips at 10, 20, 50 Hz when max_age >= 0.15; 20 Hz gives 3 periods per 0.15 s |
| Roster max_age_sec | 0.15 s | lowest tested with margin: 0.10 also had zero trips, but one 60 s run per cell |
| Aggregator period | 50 ms (fixed in code) | idle max heartbeat gap 50.4-50.9 ms |
| MCU soft stop | not a recommendation (measured at 0.2 s) | set by the firmware policy (makerspet/oomwoo-firmware#1: 150 ms); this table only shows that no soft stop occurred in any steady-state cell at 0.2 s |
| MCU CPU reset | not a recommendation (measured at 2.0 s) | set by the firmware policy (separate supervision, about five minutes initially); the 1984-1994 ms measurement confirms the implementation only |
| Stand-in clock in sim | ros | steady clock false-trips at RTF 0.25 |

Detect-to-stop budget at max_age 0.15 s, 20 Hz (kill -> zero Twist, min / median / max): 286 / 290 / 340 ms loaded (Part A),
307 / 311 / 313 ms in Gazebo. At 0.2 m/s the measured stopping distance is 0.072 m median, 0.073 m max; the Part A maximum
(0.340 s) corresponds to 0.068 m (computed: latency x speed) plus the slide. The 0.5 s default measured 576-684 ms (computed
0.12-0.14 m at 0.2 m/s, excluding the slide), below the 0.71 s bound in the spec, which was computed, not measured.

## Contract gaps seen so far
1. No defined start value for the heartbeat `sequence` (kit uses 1, matching the reference node).
2. No payload definition for `mcu_heartbeat`; the stand-in ignores the content and uses arrival only.
3. No QoS depth for heartbeat topics (kit uses depth 10, as the aggregator does).
4. A timer-driven heartbeat hides a stalled work loop (test i). The contract already requires work-path heartbeats, but nothing
   on the consumer side can detect a producer that ignores the rule; it has to be enforced by review or a shared helper.
5. Aggregator max_age and MCU soft stop add in series; the aggregator default of 0.5 s gives about 0.71 s worst case (computed).
6. A frozen or paused ROS clock must be treated as a fault by any ROS-time watchdog (test j); a steady-clock one false-trips in a slow sim.
7. The aggregator publishes on a fixed 50 ms timer; the period is neither configurable nor in the contract.

## Limits
Sample sizes: one 60 s run per Part A cell, 5 kills per cell, 5 stopping runs, one run per RTF/clock pair, 7 M2 runs; no
confidence intervals. One machine and container; other containers were running. Software rendering was not exercised (sensors off).
The stand-in is Python on the same host, not firmware: MCU timing, UART latency and motor-driver behaviour are not represented.
Stopping distance covers one speed and one drive model, in which post-command travel is chassis slide, not a deceleration ramp;
with a 1 m/s^2 ramp it would be at least 20 mm (v^2 / 2a). Latencies are timestamped on receipt by a Python probe that is itself
under load. Trips during component restart are counted only in the final kill pass (55 cycles). Part B injected only SIGKILL. Forced RTF is a
uniform slowdown, not a bursty one.
