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
- Recon wave 1 done 22:07 (valid both; coverage nav2: container CPU 4.093 cores without GPU vs 4.064 with --gpu, same RTF 0.9997): GPU nominally cheaper (-0.029
  core, 0.7%, within noise) -> rule says use it: all later runs use GPU on both instances (OHK_GPU_INSTANCES="1 2" tools/sim.sh up 2; run ... --gpu).
  coverage_planner nav2 beat period 0.200 s (max 0.202), recommended max_age 0.6.
- Recon wave 2 started 22:10: dock (inst 1) + coverage reactive (inst 2), GPU, traces beat_trace_dock_gpu / beat_trace_coverage_reactive_gpu.
