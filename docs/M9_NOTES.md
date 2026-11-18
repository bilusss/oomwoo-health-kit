# M9 working notes (progress log; resume point if interrupted)

- 2026-11-18 Part 1 done: oomwoo-ros2-tools branch health-in-node-heartbeats, commits 44717b0 (package), 689e057 (dock), 40552ff (coverage).
  colcon test in the overlay: oomwoo_health_beat 16, oomwoo_dock 15, oomwoo_coverage 21 passed (52 tests, 0 failures).
- Part 2 harness: sim.sh overlay-build / OHK_OVERLAY=1 up; run.py tasks *_inode(_recon); jobs.CoverageJob prestart (planner must beat before the
  loop can arm); beat_trace.py / beat_stats.py. In-node stamps are on the node clock (sim time) so these tasks require --health-clock sim.
