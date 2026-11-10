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
| MCU soft stop | 0.2 s (unchanged) | not swept; no soft stop in any steady-state cell; zero margin for a steady clock at RTF 0.25 |
| MCU CPU reset | 2.0 s (unchanged) | not swept; the 1984-1994 ms measurement confirms the implementation only |
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
