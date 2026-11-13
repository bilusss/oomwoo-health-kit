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
