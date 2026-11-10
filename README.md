# oomwoo-health-kit

Verification kit for the [OOMWOO](https://github.com/makerspet/oomwoo) `health-monitor` RFC
(stack health monitor and software watchdog).

It is not another aggregator. It supplies the half of the loop that a stack-health aggregator
needs in order to be tested: an MCU stand-in (watchdog state machine plus a `cmd_vel` gate), a
work-path heartbeat helper, fault-injection launch tests on real DDS, and benchmarks. Everything
speaks the published stack watchdog contract, so any aggregator candidate can be scored. The
system under test so far is [xbattlax/health-monitor](https://github.com/xbattlax/health-monitor)
at commit `b2b9796`, unmodified.

## Results so far

Full numbers, method and limits are in [docs/FINDINGS.md](docs/FINDINGS.md).

- The aggregator plus the stand-in stopped `cmd_vel` in every injected fault: critical node
  killed, process frozen, work loop stalled, node never started, aggregator killed, no roster.
- Fault to zero velocity took 286-340 ms with `max_age_sec` 0.15 s under CPU load, and
  576-684 ms with the aggregator's 0.5 s default.
- In Gazebo at 0.2 m/s the robot travelled 7.2-7.3 cm after a critical node was killed.
- A watchdog that times heartbeats on a wall clock false-trips when the simulation runs at a
  quarter of real time; one on the ROS clock does not, but must treat a frozen clock as a fault.
- A component that beats from a timer stays "healthy" with its work loop stalled. Nothing
  downstream can detect that.

## Layout

- `oomwoo_health_kit/mcu_standin.py` - ROS-free core: UNARMED / RUN / SOFT_STOP / CPU_RESET
- `oomwoo_health_kit/heartbeat.py` - `WorkHeartbeat` payload builder
- `oomwoo_health_kit/*_node.py`, `ros_heartbeat.py`, `launch/` - ROS 2 adapters and demo launch
- `test/` - unit tests and `test/launch/` fault-injection scenarios a-j
- `bench/`, `results/` - load, sim-clock and stopping-distance measurements and raw data
- `docs/SPEC.md` - design and milestones

## Run

Host tests (Python 3.12, no ROS needed):

    uv run --no-project --with pytest pytest -q

ROS tests run in Docker (`makerspet/oomwoo:jazzy-dev`; the pinned aggregator is cloned inside):

    tools/dev.sh build    # colcon build in the ohk-dev container
    tools/dev.sh test     # unit + launch tests; latencies in $OHK_RESULTS
    OHK_AGG_REF=<sha> tools/dev.sh build && OHK_AGG_REF=<sha> tools/dev.sh test   # another aggregator ref, e.g. a PR head
    tools/dev.sh shell    # then: ros2 launch oomwoo_health_kit health_loop_demo.launch.py
    tools/dev.sh down

## Status and limits

The stand-in timeouts (150 ms soft stop, 300 s CPU-reset supervision) are experimental simulation settings that
follow the firmware bring-up policy in makerspet/oomwoo-firmware#1: an MCU-local 150 ms safe-stop deadline, no CPU
reset on a short heartbeat gap, and CPU-reset supervision as a separate mechanism (about five minutes initially).
The real cutoff and reset policy belongs to the firmware / IO-board contract. Timing on the ROS clock is a simulation
convenience, not equivalent to the MCU's own clock. The aggregator under test is selectable with `OHK_AGG_REF`.

Simulation only. The stand-in is Python on the same host, not firmware, and the sample sizes
are small; see the Limits section of the findings.

Built with substantial AI assistance (Anthropic's Claude, via Claude Code), directed by the
repository owner.

License: Apache-2.0.
