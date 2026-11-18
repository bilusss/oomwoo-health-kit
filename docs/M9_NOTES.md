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
- Wave 2 22:10: dock recon valid (1801 sim s, 57 goals, RTF 0.978, container 3.08 cores, 4 soft stops all 'end / drive_supervision' = harness set_pose); reactive recon on inst 2 INVALID:
  gz sim SIGSEGV (exit 139) 19 s after start when both GPU stacks started within seconds of each other (raw record kept: m9/2/sessions.jsonl line 2). Retried alone 22:38
  (label m9_recon_retry), trace beat_trace_coverage_reactive_gpu_retry.jsonl. LESSON: stagger GPU stack starts (OHK_RUN_STAGGER_S / manual sleep).
- Dock final roster written (config/nav_rosters_dock_inode.yaml: detector 1.8 s, drive 0.15 s). Fault chain inst 1 started 22:40 (tools/m9_chain.sh, args in
  results/fullstack/m9/_chains/faults_inst1.txt): dock_detector kill x20, stop x10, dock_drive kill x10; label m9_faults. NOTE the reactive recon overlaps with these
  trials' stack starts (load spikes) -> its max gaps are conservative.
- TODO after reactive recon: roster nav_rosters_amcl_inode.yaml (coverage_planner, nav2: 0.6 s), reactive value from data; fault chain inst 2 (coverage kill 10/stop 5 nav2 + reactive);
  then soaks (>= 6 sim h: dock 5x1800, nav_amcl 2x1800 inst 1; coverage nav2 3x1800 + reactive 3x1800 inst 2); then docs/M9_RECON.md, tests, commit.
- 22:47 GPU ABANDONED: containers lose GPU access after ~30 min (docker exec nvidia-smi: "Failed to initialize NVML: Unknown Error" in ohk-sim-2; wave-1 gpu_mid/gpu_at_end
  snapshots show the same), after which a stack start cannot render (gazebo "Unable to load Ogre Plugin RenderSystem_GL3Plus", SIGSEGV 139): reactive recon crashed twice in
  ohk-sim-2 (22:08, 22:39), the dock fault chain would have broken the same way. The GPU saving was 0.029 core (0.7%) anyway. Everything from here on runs on CPU rendering
  (containers recreated without GPU). Dock chain: 9 valid GPU-rendered sigkill_dock_detector trials (m9/1/faults.jsonl trials 1-9, trial 10 aborted by my SIGTERM) are kept;
  the chain restarted on CPU 22:55 with --reps 11 so that the kill series reaches 20 valid. Chain file: results/fullstack/m9/_chains/faults_inst1_cpu.txt.
  Reactive recon (CPU) on inst 2 started 22:53, label m9_recon_retry2, trace beat_trace_coverage_reactive_cpu.jsonl (invalid starts: m9/2/sessions.jsonl lines 2-3).
