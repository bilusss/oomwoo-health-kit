# M8 recon: what the watcher can see behind a command stage that keeps emitting, and behind a reversing controller

Date 2026-11-15. Same image and stacks as M7 (`makerspet/oomwoo:jazzy-dev`; `docs/M7_RECON.md`). Rule from M7 kept: the live graph and the
rates are read before a rule is written. New tool: `bench/fullstack/probe_trace.py` records every message (receive time on the host
monotonic clock) of the command chain, the dock topics and the Nav2 action status topics of a running stack; `bench/fullstack/m8_analysis.py`
turns the traces into the tables below and replays the new cores over them. Raw traces (kept): `results/fullstack/m8_recon/`.

| Trace | Stack / workload | Length | Contents |
|---|---|---|---|
| `trace_recovery_1.jsonl` | `recovery` task (AMCL stack, `composition false`, BT recoveries with a wall), 1 of 2 instances running | 469 s | 2242 `/cmd_vel_nav`, 8999 `/cmd_vel_smoothed` messages; 15 NavigateToPose, 6 spin, 3 backup, 3 wait goals |
| `trace_dock_a/b/c.jsonl` | `dock` task (kitchen_dining, no Nav2), 2 instances running | 759 s, 1079 s, 1078 s | 2435 `/dock_detector/dock_pose` messages, 89 reversing episodes |

Commands: `python3 bench/fullstack/m8_analysis.py chain results/fullstack/m8_recon/trace_recovery_1.jsonl`, `... dock results/fullstack/m8_recon/trace_dock_?.jsonl`,
`... derive results/fullstack/m8_recon/trace_recovery_1.jsonl --dock results/fullstack/m8_recon/trace_dock_?.jsonl`.
Receive times include the probe's own scheduling delay (a few ms at this load).

## 1. Gap A: the command chain behind Nav2's velocity smoother

### Graph (M7 `graph_nav_amcl.json`, unchanged)
`controller_server` and 5 `behavior_server` publishers write `/cmd_vel_nav` (RELIABLE, VOLATILE); `velocity_smoother` subscribes it and writes
`/cmd_vel_smoothed` (only publisher); `collision_monitor` passes that to `/cmd_vel` (the stand-in input). So the smoother's input has a different
topic from the stand-in's: behavior_server does NOT write the stand-in's topic, it shares `/cmd_vel_nav` with the controller. A rule on
(`/cmd_vel_nav`, `/cmd_vel_smoothed`) therefore covers both commanders with one pair of topics and ignores writers that bypass the smoother
(`docking_server`, the reactive coverage planner), which `drive_supervision` already covers.

### Rates and behaviour during navigation and recoveries (`m8_analysis.py chain`, 469 s)
Period between consecutive non-zero `/cmd_vel_nav` messages, by the Nav2 action that was active (recovery actions win over navigate_to_pose):

| Source | Messages | Period p50 / p99 / max (s) |
|---|---|---|
| navigate_to_pose (controller_server) | 1941 periods | 0.200 / 0.200 / 0.201 |
| spin (behavior_server) | 92 | 0.100 / 0.101 / 0.101 |
| backup (behavior_server) | 42 | 0.100 / 0.100 / 0.100 |
| no action active | 44 | 0.200 / 0.200 / 0.200 |
| wait | no message at all while a wait is active (non-zero `/cmd_vel_smoothed` samples while a wait was active: 0) | - |

So the controller commands at 5 Hz and the recovery actions at 10 Hz, both with sub-millisecond-level regularity at this load (max 201 ms). The smoother re-emits at 20 Hz
(`/cmd_vel_smoothed`: 8999 messages in 469 s). Age of the newest non-zero input at each non-zero smoothed sample, while the last input was non-zero: max 0.195 s
(navigate), 0.093 s (spin), 0.090 s (backup).

Stops (all 86 zero input messages came from the commanders: every stop that was seen ended with a zero message on `/cmd_vel_nav`; there is no stop without one in this
trace): time from the first zero input message to the first zero smoothed message: navigate n=70 p50 0.043 / max 0.245 s; spin n=5 p50 0.240 / max 0.344 s;
backup n=2 0.090 s; no action n=1 0.226 s (the smoother's `max_decel` -2.5 m/s^2 and -3.2 rad/s^2). A zero input is the commander's way to say stop; the smoother needs up to
0.34 s to follow. n is small for spin and backup (5 and 2): the bound below is a measured maximum scaled by 1.5, not a tail estimate.

### Derived bounds (formulas fixed before any M8 trial or soak; `m8_analysis.py derive`)
- `in_max_age_s` = ceil to 50 ms of 1.5 x the longest period between non-zero inputs (0.201 s) = **0.35 s**.
- `stop_grace_s` = ceil to 50 ms of 1.5 x the longest time from the first zero input to the first zero smoothed message (0.344 s) = **0.55 s**.
Replaying `CommandChainGuard` with these bounds over the trace: 1 not-ok interval, 0.15 s long, 2.7 s after the trace started, "stage output non-zero, no input ever
seen" (the trace began in the middle of a motion; an artifact of the recording, not of a run). Nothing in the remaining 466 s.

### What the rule can and cannot do (before measuring)
The earliest it can act is `in_max_age_s` after the last input: 0.35 s minus the age of the last input at the moment of the fault, plus the roster `max_age_sec` (0.15 s)
and the stand-in's 0.15 s. The smoother's own timeout is 1.0 s, so the best case is a bit more than half of the M7 stopping time. A tighter bound would need the
commander's rate (10 Hz in the recovery actions, 5 Hz in the controller) and would false-trip the controller on a stall of one period.

## 2. Gap B: the dock detector while dock_drive reverses

### Graph and source
From the dock stack (`dock_sim`, 2 instances running): `dock_drive` is the only writer of `/cmd_vel` (20 Hz in every state, zeros included), reverses only in state BACK at
-0.06 m/s (`v_back`, source `oomwoo_dock/dock_drive_node.py`) and carries its estimate by odometry for `pose_timeout_s` = 3.0 s after the last fix, then holds. The fix is
`/dock_detector/dock_pose` (`geometry_msgs/PoseStamped`, the detector's `detect_hz` 2.0 against a 5 Hz scan). `/dock_drive/state` (`std_msgs/String`) is published
**only on a state change** (a watcher starting after the change, or a restarted dock_drive, would not know the state), so the rule infers reversing from the command:
held linear x <= -0.02 m/s. In 89 of 89 reversing episodes the state at the start was BACK; GOTO / REGROUP never reversed in the recon.

### Fix timing while reversing (`m8_analysis.py dock`, 3 traces, 89 episodes of 6.5-6.85 s)
| Quantity | n | p50 | p95 | p99 | max |
|---|---|---|---|---|---|
| gap between consecutive fixes inside an episode (s) | 844 | 0.600 | 0.606 | 1.202 | **2.403** |
| age of the newest fix at each reverse command (20 Hz samples) (s) | 11909 | 0.342 | 0.599 | 1.143 | 2.395 |
| age of the newest fix at the start of an episode (s) | 89 | 0.250 | 0.590 | 0.790 | 0.790 |
| time from the start of an episode to its first fix (s) | 89 | 0.354 | 0.555 | 0.562 | 0.562 |
| age of the newest fix at the end of an episode (s) | 89 | 0.342 | 1.043 | 2.343 | 2.343 |

The fix is regular at 0.60 s; a missing fit makes a 1.2 s gap in 32 of 844 gaps (3.8%). Fix gaps come in steps of the 0.6 s period (0.6, 1.2, 1.8, 2.4 s). Three episodes have a longer
silence: in two (traces a and b) the fixes stopped 2.04 s and 2.34 s before the end of the reverse (the end of the approach into the bay); in one (trace c) there is a 2.4 s gap in
the MIDDLE of the episode (1.66 s to 4.06 s after its start), not at the end. `dock_drive` carried on in all three and the state DONE was seen 89 times (once per episode).

### Derived bounds (formulas fixed before any M8 trial or soak; `m8_analysis.py derive`)
- `in_max_age_s` = ceil to 50 ms of 1.5 x the longest period between non-zero inputs (0.201 s) = **0.35 s**.
- `stop_grace_s` = ceil to 50 ms of 1.5 x the longest time from the first zero input to the first zero smoothed message (0.344 s) = **0.55 s**.
Replaying `CommandChainGuard` with these bounds over the trace: 1 not-ok interval, 0.15 s long, 2.7 s after the trace started, "stage output non-zero, no input ever
seen" (the trace began in the middle of a motion; an artifact of the recording, not of a run). Nothing in the remaining 466 s.

### What the rule can and cannot do (before measuring)
The earliest it can act is `in_max_age_s` after the last input: 0.35 s minus the age of the last input at the moment of the fault, plus the roster `max_age_sec` (0.15 s)
and the stand-in's 0.15 s. The smoother's own timeout is 1.0 s, so the best case is a bit more than half of the M7 stopping time. A tighter bound would need the
commander's rate (10 Hz in the recovery actions, 5 Hz in the controller) and would false-trip the controller on a stall of one period.

## 2. Gap B: the dock detector while dock_drive reverses

### Graph and source
From the dock stack (`dock_sim`, 2 instances running): `dock_drive` is the only writer of `/cmd_vel` (20 Hz in every state, zeros included), reverses only in state BACK at
-0.06 m/s (`v_back`, source `oomwoo_dock/dock_drive_node.py`) and carries its estimate by odometry for `pose_timeout_s` = 3.0 s after the last fix, then holds. The fix is
`/dock_detector/dock_pose` (`geometry_msgs/PoseStamped`, the detector's `detect_hz` 2.0 against a 5 Hz scan). `/dock_drive/state` (`std_msgs/String`) is published
**only on a state change** (a watcher starting after the change, or a restarted dock_drive, would not know the state), so the rule infers reversing from the command:
held linear x <= -0.02 m/s. In 89 of 89 reversing episodes the state at the start was BACK; GOTO / REGROUP never reversed in the recon.

### Fix timing while reversing (`m8_analysis.py dock`, 3 traces, 89 episodes of 6.5-6.85 s)
| Quantity | n | p50 | p95 | p99 | max |
|---|---|---|---|---|---|
| gap between consecutive fixes inside an episode (s) | 844 | 0.600 | 0.606 | 1.202 | **2.403** |
| age of the newest fix at each reverse command (20 Hz samples) (s) | 11909 | 0.342 | 0.599 | 1.143 | 2.395 |
| age of the newest fix at the start of an episode (s) | 89 | 0.250 | 0.590 | 0.790 | 0.790 |
| time from the start of an episode to its first fix (s) | 89 | 0.354 | 0.555 | 0.562 | 0.562 |
| age of the newest fix at the end of an episode (s) | 89 | 0.342 | 1.043 | 2.343 | 2.343 |

The fix is regular at 0.60 s; a missing fit makes a 1.2 s gap in 32 of 844 gaps (3.8%); one gap of 2.4 s. The three longest ages (2.04 s, 2.34 s, 2.40 s) occurred in
three different episodes, near the end of the reverse (the last 1-2 s), while dock_drive carried on and docked (all 89 episodes ended in DONE).

### Derived bound
`fix_max_age_s` = ceil to 50 ms of 1.5 x the 99th percentile of the in-episode gaps (1.202 s) = **1.85 s** (strict: the newest fix must be fresh from the first reversing
command on; the ages at episode starts are at most 0.79 s, so `from_reverse_start` is not needed). This was a choice: rule A uses 1.5 x the MAXIMUM period (0.201 s), rule B 1.5 x the p99,
because 1.5 x the maximum gap (2.403 s) would be 3.65 s, beyond dock_drive's own 3.0 s hold, and the rule would do nothing. Since the gaps come in 0.6 s steps, 1.85 s sits 50 ms above three
periods (1.8 s): a silence of three missing fits trips it, two do not.
Replaying `ReverseFixGuard` with it over the three traces: 3 not-ok intervals in 89 episodes (one per trace, exactly the three silences above; 0.15 s, 0.45 s, 0.51 s long; the
warning always reads about 1.86-1.89 s because it is logged at the first failing 50 Hz check). So the rule is expected to withhold the heartbeat in about 3% of the attempts; the first interval (0.15 s)
is at the limit of what the roster's `max_age_sec` 0.15 s turns into a stop. A bound that covers every observed age (2.5 s) would leave only 0.5 s of the 3.0 s the dock_drive coast gives, so the
rule is closer to "shorten the blind coast from 3.0 s to about 1.85 s + 0.35 s" than to "detect at once". The soak count and the fault trials are in `docs/FINDINGS.md` (M8).

**Precondition of rule B: `dock_drive` is the only reverser on the watched command stream.** "Reversing" is inferred from ANY writer of the command topic (held linear x <= -0.02 m/s). In the dock stack
`dock_drive` is the only writer of `/cmd_vel` (graph facts above) and `nav_watch_dock_fix.yaml` is the only config that selects the rule. On a stack where another node reverses (Nav2 BackUp at about -0.1 to
-0.15 m/s for about 3 s, teleop) the rule would trip 1.85 s into that reverse, because no dock fix exists. A unit test documents this (`test_reverse_by_any_writer_trips_without_a_fix`).

## 3. Item C (coverage planner, nav2 executor) and item D (reactive command period)
- C: nothing to implement. With `executor:=nav2` the planner only sends NavigateToPose goals; a dead planner leaves a working controller and a robot that finishes the waypoint.
  Whether the job is still running is a task-level question (the planner's `cleaning_active` and `~/plan` topics), not a motion-safety one, and no watch was built.
- D: measured with one instance running (4 sessions, 12.9 min of planner activity, `trace_reactive_{a,b}.jsonl`): period p50 0.200 s, 0.400 s in about 7% of the periods (missed ticks), max 0.601 s. Table and caveats in FINDINGS (M8, Item D).

## 4. Not done
- No upstream node was changed and none was asked to publish anything new.
- Only the AMCL/full-Nav2 stack and the `dock_sim` dock stack were examined; the opennav `docking_server` (second writer of `/cmd_vel`) was not run.
- Nav2 BackUp goes through the smoother (rule A would see a dead BackUp commander) but it is a reversing writer on `/cmd_vel` for rule B: B must not be used on a stack that has one (see the precondition in section 2). The `/cmd_vel_nav` rate during DIRECT spin actions was not traced (the chain trace has BT spins and backups only).
