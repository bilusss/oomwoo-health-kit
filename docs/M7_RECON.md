# M7 recon: AMCL, coverage, docking and recoveries on the OOMWOO sim stack

Date 2026-11-14. Image `makerspet/oomwoo:jazzy-dev` (`/ros_ws`), upstream clones read from GitHub (makerspet/oomwoo, oomwoo-one,
oomwoo-install, xbattlax/health-monitor) into /tmp. Method rule: before a watcher config is written, the live graph is read
(`bench/fullstack/graph_facts.py`: publishers, subscriptions, QoS per topic; `probe_rates.py`: rates moving and stationary). A watcher on
the wrong topic is a fail-open defect. Raw output: `results/fullstack/m7_recon/`. Rates are wall Hz = sim Hz (RTF 1.0).

## 1. What exists upstream, per task

The workspace `/ros_ws/src/oomwoo-ros2-tools` (oomwoo_clean, oomwoo_coverage, oomwoo_dock, oomwoo_sim_support, oomwoo_localization, ...) is
what the image ships; the `makerspet/oomwoo` repo itself holds docs and contributions only.

| Task | Upstream support | Used in M7 |
|---|---|---|
| AMCL localization | `oomwoo_clean nav.launch.py` (default `localization:=amcl`): `nav2_bringup bringup_launch.py` with map_server + AMCL + the full Nav2 (controller, smoother_server, planner, behavior_server, bt_navigator, velocity_smoother, collision_monitor, docking_server, route_server, waypoint_follower) in ONE composed `nav2_container`; `initialpose_pub` seeds AMCL at the spawn pose. Map: `oomwoo_sim_support` `living_room.yaml` (world aligned). | yes, unmodified, plus the same call with `use_composition:=False` for per-node faults |
| Coverage / cleaning | `oomwoo_coverage coverage_planner` (boustrophedon, 234 waypoints on living_room) with `executor:=nav2` (one NavigateToPose per waypoint) or `executor:=reactive` (drives `/cmd_vel` itself, transits through Nav2); `oomwoo_sim_support coverage_meter` + `ground_truth` score it and gate `stop_at_target` (0.90). `coverage_regression.launch.py` uses a trimmed Nav2 (no smoother, no collision monitor) on test_room; not used, the coverage nodes are started on the full AMCL stack with the same parameters. `oomwoo_clean cleaning.launch.py` is only a goal_pose to NavigateToPose bridge. Wall-follow / contour / edge / bump-map launches exist (not run). | yes, both executors |
| Docking | `oomwoo_sim_support dock_sim.launch.py` = `ir_beacon_sim` + `oomwoo_dock` `dock_detector` (LiDAR fit) + `dock_drive` (SEARCH, GOTO, TURN, BACK, REGROUP, DONE); only `kitchen_dining.world` has the `vacuum_dock` model. No Nav2 needed. | yes |
| Nav2 `opennav_docking` | The `docking_server` runs in the stack, but `oomwoo_one` navigation.yaml configures only the template: `SimpleChargingDock` named `nova_carter_dock` with placeholder dock poses (map 0,0 / 10,10 / 30,30), no dock model in living_room, no charging/battery signal in the sim. **Not run: no upstream sim docking through opennav_docking exists.** It does appear as a second writer of `/cmd_vel`. | no (reason stated) |
| BT recoveries | Nav2's `navigate_to_pose_w_replanning_and_recovery.xml` (clear costmaps, spin, wait, backup in a round-robin recovery branch) is the stock BT in `oomwoo_one` navigation.yaml. `oomwoo_sim_support spawn_obstacle.launch.py` places an off-map box. | yes: M7 places a box across the path through the gz service |

## 2. Graph facts per stack

### 2.1 AMCL stack (`localization:=amcl`), composed (`graph_nav_amcl.json`)
- `/cmd_vel` (the stand-in's input): 2 writers at rest, `collision_monitor` and `docking_server` (RELIABLE, VOLATILE); only subscriber of
  `/cmd_vel_gated` is `ros_gz_bridge`. `/cmd_vel_nav`: `controller_server` + 5 publishers of `behavior_server` -> `velocity_smoother`
  -> `/cmd_vel_smoothed` -> `collision_monitor` -> `/cmd_vel` (all RELIABLE). `/cmd_vel_teleop` is subscribed by behavior_server, no publisher.
- `/scan`, `/odom`: one writer (`ros_gz_bridge`). `/tf`: three writers (bridge, robot_state_publisher, `amcl`); `map->odom` comes from amcl.
  `/amcl_pose` RELIABLE + TRANSIENT_LOCAL, `/particle_cloud` BEST_EFFORT. Footprints: one writer each (RELIABLE, TRANSIENT_LOCAL).
- Rates (100 s, goals running 59 s / stationary 41 s, `rates_nav_amcl.json`):

| Signal | Moving | Stationary | Max gap moving | Note |
|---|---|---|---|---|
| `/scan` | 4.97 Hz | 5.04 Hz | 202 ms | |
| `/odom` | 49.99 Hz | 49.92 Hz | 22 ms | |
| `map->odom` new stamps | 4.97 Hz | 5.01 Hz | 206 ms | amcl sends the transform from its scan callback |
| `/amcl_pose` | 4.77 Hz | 0 (3 msgs in 41 s) | 1.2 s | silent when idle: NOT a liveness signal |
| local costmap footprint | 5.01 Hz | 4.96 Hz | 200 ms | |
| global costmap footprint | **2.00 Hz** | 2.00 Hz | 500 ms | 5.0 Hz on the slam stack: roster max_age 1.5 s |
| `/cmd_vel_nav` | 4.97 Hz | 0.07 | 217 ms | only while a goal runs |
| `/cmd_vel_smoothed`, `/cmd_vel` | 20.0 Hz | idle: silent | 51 / 61 ms | |

- Non-composed variant (`composition:=false`): same nodes, one process each; same topic set (checked by the fault trials, which found
  `amcl`, `controller_server`, `behavior_server` by their own pids). Caveat found: `nav2_bringup` loaded `map.yaml` from the params file
  instead of the `map` argument in this mode, so the launch writes a copy of `navigation.yaml` with the absolute map path.

### 2.2 Coverage (`graph_coverage_nav2.json`)
- Adds `coverage_planner` as a writer of `/cmd_vel` (3 writers) in BOTH executors, and publishes `/coverage_planner/cleaning_active`,
  `~/plan`; subscribes `/coverage_meter/ratio`. With `executor:=nav2` the commanding path is bt_navigator -> controller (planner only sends
  NavigateToPose goals); with `executor:=reactive` the planner publishes `/cmd_vel` itself for sweep passes.
- Reactive executor command rate (`rates_coverage_reactive.json`, 90 s): `/cmd_vel` moving p50 59 ms, p95 201 ms, max 405 ms (Nav2's 20 Hz chain plus the
  planner's own 5 Hz passes); `/cmd_vel_nav` 2.8 Hz, `/cmd_vel_smoothed` 11.5 Hz. The probe ran at normal load.
- The planner reports `complete` and stops moving after about 520-1010 s of a session (see FINDINGS).

### 2.3 Dock (`localization:=none`, kitchen_dining, `rates_dock.json`)
- Writers of `/cmd_vel`: `dock_drive` only. It publishes at 20 Hz in EVERY state, including zero commands when IDLE/DONE/disabled and when
  it has lost the dock fix ("holding"). `/scan` 4.96 Hz, `/odom` 50.0 Hz. `/dock_detector/dock_pose` 1.5 Hz moving, seconds-long silences:
  not watched.
- The dock needs no map and no TF tree beyond odom; roster = lidar, odom, drive_supervision.
- Harness note: the first dock rate probe used the `gz` CLI for teleports and shows stationary gap maxima of 0.34-0.38 s on `/odom` and `/cmd_vel`
  (`rates_dock.json`). The same 340 ms stationary `/odom` maximum appears in `rates_coverage_reactive.json`, which had no teleports, so the CLI
  is not isolated as the cause. The harness now uses the gz-transport python bindings (persistent client) because the CLI costs about 2 s of start-up per call.

## 3. Watch configs per task (reuse of cores, no code change in `topic_watch`)
`config/nav_watch_amcl.yaml` + `nav_rosters_amcl.yaml` (nav_amcl, coverage, coverage_reactive, recovery),
`config/nav_watch_dock.yaml` + `nav_rosters_dock.yaml` (dock). Same watch types as M6; `localization` = `map->odom` tf watch;
`drive_supervision` = motion_consistency on `/cmd_vel` (any writer) + `/odom`, thresholds unchanged from M6 (cmd_max_age 0.15 s,
coast_max 0.5 s). Thresholds were NOT tuned after a run; see FINDINGS for the rules that tripped.
