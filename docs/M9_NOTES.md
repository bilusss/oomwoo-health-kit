# M9 working notes (progress log; resume point if interrupted)

- 2026-11-18 Part 1 done: oomwoo-ros2-tools branch health-in-node-heartbeats, commits 44717b0 (package), 689e057 (dock), 40552ff (coverage).
  colcon test in the overlay: oomwoo_health_beat 16, oomwoo_dock 15, oomwoo_coverage 21 passed (52 tests, 0 failures).
- Part 2 harness: sim.sh overlay-build / OHK_OVERLAY=1 up; run.py tasks *_inode(_recon); jobs.CoverageJob prestart (planner must beat before the
  loop can arm); beat_trace.py / beat_stats.py. In-node stamps are on the node clock (sim time) so these tasks require --health-clock sim.
- FINDING 1 (smoke, results/fullstack/m9/_smoke1): in-node beats stamped with SIM time + aggregator/watcher on sim time gave 28 soft stops in 240 sim s
  (20 blamed on dock_drive, max_age 5 s): the aggregator treats age < 0 as stale and a 20 Hz timer beat is stamped on the very /clock tick that the
  aggregator's own /clock may not have reached yet. Fix (harness only): bench/fullstack/beat_relay.py (nodes publish to /oomwoo/health/component_node,
  relay re-stamps onto the wall clock, age = max(0, sim_now - stamp)); health stack stays on WALL time as in M7/M8; tasks *_inode* require --health-clock wall.
  Smoke 2 with relay (results/fullstack/m9/_smoke2): 0 soft stops in 301 sim s, container CPU 3.16 cores (sim-clock variant 4.01).
- Recon wave 1 started 2026-11-18 ~21:45: coverage nav2 executor, inst 1 (cpu) + inst 2 (--gpu), 1800 sim s each, traces beat_trace_coverage_nav2_{cpu,gpu}.jsonl.
  Env for every sim.sh call: export OHK_RES_SUB=m9 OHK_OVERLAY=1 ; GPU only on instance 2 via OHK_GPU_INSTANCES=2.
