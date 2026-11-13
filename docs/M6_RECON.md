# M6 recon: health loop on the real OOMWOO sim stack

Date 2026-11-13. Image makerspet/oomwoo:jazzy-dev, workspace /ros_ws. Recon only, no code. Throwaway containers
(software-GL one, GPU one) were created and removed; ohk-dev untouched. Everything marked UNVERIFIED was not measured.
Host: 12 threads, RTX 5070 Ti (about 1 GB VRAM was free, other containers hold the rest).

## 1. Launching the stack

Two stacks exist; both start headless with no display. The recommended one for M6 is B.

A. oomwoo_sim_support (CI harness, trimmed Nav2: controller, planner, behavior, bt_navigator, AMCL; no velocity
   smoother, no collision monitor; controller publishes /cmd_vel directly; lifecycle bond_timeout 0.0 = bonds off).
```
ros2 launch oomwoo_sim_support coverage_regression.launch.py      # sim_bringup + coverage_planner + coverage_meter, test_room
ros2 launch oomwoo_sim_support sim_bringup.launch.py              # world+bridge+AMCL+Nav2, no task node (args: world map x_pose y_pose yaw with_nav gui robot_model)
```
B. Real bringup path (what a user runs): two terminals, WORKS headless (measured).
```
ros2 launch oomwoo_gazebo world.launch.py headless:=true robot_model:=oomwoo_one world:=living_room.world \
    [enable_lidar|enable_ranges|enable_tof|enable_cameras|enable_imu:=true|false] [odom_source:=ground_truth|robot_wheels]
ros2 launch oomwoo_bringup navigation.launch.py robot_model:=oomwoo_one use_sim_time:=true x_pose:=-2.0 y_pose:=-0.5 \
    map:=/ros_ws/install/oomwoo_gazebo/share/oomwoo_gazebo/maps/living_room.yaml
```
   navigation.launch.py localizes with slam_toolbox (localization_slam_toolbox_node) because living_room_serial.posegraph
   exists next to that map; otherwise it falls back to AMCL (`localization:=amcl`). Composed Nav2 in ONE process
   (component_container_isolated "nav2_container"): map_server, controller, planner, behavior, bt_navigator,
   velocity_smoother, collision_monitor, two lifecycle managers. It always also starts rviz2, which dies with
   "could not connect to display"; harmless (the launch keeps running).
   Alternative without rviz, AMCL, full nav2_bringup (adds docking_server, route_server, smoother_server,
   waypoint_follower; also WORKS, 7/7 goals):
```
ros2 launch oomwoo_clean nav.launch.py robot_model:=oomwoo_one     # default map = oomwoo_sim_support maps/living_room.yaml, spawn -2.0,-0.5
ros2 launch oomwoo_clean cleaning.launch.py                         # adds clean_to_goal (goal_pose -> NavigateToPose); coverage:=true adds meter
```
Task launch files that exist (package:file): map: oomwoo_bringup:navigation.launch.py `slam:=True`, oomwoo_bringup:explore.launch.py
(explore_lite), oomwoo_bringup:cartographer.launch.py; clean: oomwoo_sim_support:coverage_regression.launch.py,
oomwoo_clean:{cleaning,nav,wall_clean,wall_clean_bump_out,contour_follow,edge_clean,bump_map}.launch.py;
navigate: oomwoo_bringup:navigation.launch.py; dock: oomwoo_sim_support:dock_sim.launch.py (ir_beacon_sim + oomwoo_dock:dock.launch.py);
localization: oomwoo_localization:{localization_health,localization_recovery}.launch.py.
Headless selection: `headless:=true` (world.launch.py) or `gui:=false` (sim_bringup, default). Both FORCE
LIBGL_ALWAYS_SOFTWARE=1 and GALLIUM_DRIVER=llvmpipe in the launch file.
Verified to run: world.launch.py, bringup navigation.launch.py (slam_toolbox), clean nav.launch.py (AMCL), coverage_regression.
UNVERIFIED (not run): dock_sim, explore, cartographer, slam:=True mapping, wall_clean/contour/edge, localization_* launches.
Pitfall: killing `ros2 launch` leaves the composed container and lifecycle processes alive; kill by name (kill -9) between runs,
else duplicate nodes appear on the same DDS domain.

## 2. Nodes and work-cycle topics (stack B, slam_toolbox variant, robot driving, RTF 1.0, software GL; 12-20 s windows)

Rates are wall Hz = sim Hz (RTF 1.0). Subscribed best-effort/volatile; publisher QoS shown (REL=reliable, VOL=volatile, TRAN=transient-local).
```
node (process)            work-cycle topic                      rate (moving)      pub QoS   note
ros_gz_bridge             /scan                                 5.0 Hz             REL/VOL   lidar; also /odom 50, /imu 100, /tof_front/points 15, /range_left|right 20, /joint_states 1000
robot_state_publisher     /tf (3-4 pubs share it)               75-120 Hz          REL/VOL   cannot attribute per node; /joint_states 1000 Hz
slam_toolbox (lifecycle)  /pose                                 1.6 Hz             REL/VOL   only while moving (UNVERIFIED idle); map->odom is on /tf; /map_slam 0.2 Hz TRAN
amcl (stack A / clean nav) /amcl_pose                           1.95 Hz moving     REL/TRAN  NO output when robot idle (0 msgs in 10 s); /particle_cloud same
controller_server         /cmd_vel_nav                          5.0 Hz (=controller_frequency) REL/VOL  ONLY during a goal; 6 publishers share it (behavior_server too)
                          /lookahead_point, /received_global_plan 2.7 Hz   only during a goal
velocity_smoother         /cmd_vel_smoothed                     20.0 Hz            REL/VOL   only while it gets input; idle 0
collision_monitor         /cmd_vel                              20.0 Hz            REL/VOL   pass-through: all polygons `enabled: false`, scan source enabled; idle 0
local_costmap             /local_costmap/published_footprint    5.0 Hz (update)    REL/TRAN  /local_costmap/costmap 1.67 Hz (publish 2 Hz configured)
global_costmap            /global_costmap/published_footprint   2.0 Hz             REL/TRAN  /global_costmap/costmap 1.0 Hz; *_updates topics silent
planner_server            /plan                                 0.4 Hz, event      REL/VOL   event driven only
bt_navigator              /behavior_tree_log                    63 Hz during goal  REL/VOL   0 idle
behavior_server           (none periodic)                       -                            NO periodic output; /cmd_vel_nav only in recoveries
map_server                (none; /map latched, 0 msgs)          -                            NO periodic output
lifecycle managers        /diagnostics                          2.06 Hz total (2 pubs)       see 3
all Nav2 nodes            /bond                                 132 Hz total, 24 pubs        bond_heartbeat_period 1.0 s in navigation.yaml, 2 pubs per node
coverage_planner          /cmd_vel directly (reactive executor), NavigateToPose goals otherwise; UNVERIFIED periodic topic
```
Idle (no goal) was measured in stack A/clean nav: /scan 5, /odom 50, /tf 75, costmaps and footprints continue at the rates above;
cmd_vel chain, amcl_pose, bt log all silent. Consequence: for navigate/clean, "controller alive" evidence exists only while a goal is active;
/cmd_vel_nav silence between goals is normal. The roster must change with job state, or the controller needs a task-state-aware watcher.
Other /cmd_vel publishers bypass the smoother and collision monitor: docking_server (nav2), dock_drive, wall_clean, contour_follower,
coverage_planner (reactive), kidnap_recovery, relocalize_on_lost, bumper_probe (all in oomwoo-ros2-tools sources).

## 3. Existing liveness mechanisms

- Nav2 lifecycle_manager bond: oomwoo_one navigation.yaml sets only `bond_heartbeat_period: 1.0` for all nodes (comment says 4 s bond_timeout,
  the Nav2 default; the key itself is not in the file; runtime value for stack B UNVERIFIED). Stack A launch sets `bond_timeout: 0.0` (verified at
  runtime on lifecycle_manager_navigation) = bonds disabled, `attempt_respawn_reconnection` true. Bond detects node death/hang of Nav2 lifecycle
  nodes only, in seconds (about 4 s), far slower than the 0.15 s window. Not an RFC component heartbeat.
- /diagnostics: 2 publishers, only lifecycle_manager_localization and lifecycle_manager_navigation (2 Hz); no per-node diagnostics.
  diagnostic_updater is installed but nothing in the stack uses it.
- No heartbeat topics, no watchdog, no twist_mux, no health package in the image (`ros2 pkg list` grep health|watchdog|twist_mux: nothing).
- xbattlax recovery-safety / health-monitor: NOT in the image (find for *xbattlax*, *recovery_safety*, *health_monitor* returned nothing).
  Only oomwoo_nav_localize/kidnap_recovery and oomwoo_localization/localization_health exist. Recovery-safety source is only in the repo clone
  (contributions/recovery-safety/xbattlax).

## 4. GPU

- NVIDIA EGL needs the vendor file, absent in the image: write `{"file_format_version":"1.0.0","ICD":{"library_path":"libEGL_nvidia.so.0"}}`
  to a json and set `__EGL_VENDOR_LIBRARY_FILENAMES=<it>`. The stock launch files force llvmpipe, so a copy of world.launch.py with the
  `if headless_bool or software_gl_bool:` env block disabled was used (copy in the container's /tmp, repo untouched).
- Evidence: `nvidia-smi --query-compute-apps` lists the gz process (type G) using 227 MiB. GPU utilisation about 1% (sampled once).
- Measured, living_room, enable_cameras:=true (all sensors on), stack B driving goals, single 15-20 s samples:
  - locked (default real_time_factor): RTF 1.00 on GPU, 0.99 software. gz CPU: 75% GPU vs 153% software (1 core = 100%).
  - unlocked (`gz service -s /world/default/set_physics ... --req "real_time_factor: 20"`): RTF 3.34 GPU vs 2.71 software; goals still succeed.
  - Without cameras (default), software GL: RTF 1.00, gz 75-80% CPU, nav2_container 40%.
- Lidar: yes, rendering. /scan, side ranges and ToF are all `gpu_lidar` sensors (urdf/plugins.xacro); headless needs EGL/OSMesa, llvmpipe works.
- Conclusion: software GL is enough for RTF 1.0 on this host; GPU gives about 2x lower gz CPU and about 20% more speed when unlocked.

## 5. Soak workload

- coverage_regression.launch.py (stack A, test_room, software GL, RTF about 1.0): one run reached target 90% at sim 623 s (91.5% at
  sim 691 s, path 102 m); the 313-waypoint plan was at 260/313 after about 14 min wall, remaining gap-fill not timed (run cut off). Planner
  logs "coverage complete" at the end (not observed). Terminates by itself; needs a restart loop for hours. One run only, so
  determinism between runs is UNVERIFIED (2 planner_server worldToMap ERRORs at the costmap edge, 127 WARN/ERROR lines total, run continued).
- Simpler soak: goal loop over 8 fixed poses in living_room with a NavigateToPose client (script was in /tmp, not kept): stack B slam_toolbox,
  4/4 goals in 62 s; AMCL stack 7/7 in 125 s; with cameras 5 goals in a minute. Each goal 5-30 s wall at RTF 1 (varies 4-28 s for the same
  pose pair), so it is repeatable in outcome, not in timing. Goal loop gives continuous controller activity; coverage gives variety.

## 6. Proposed design (draft)

1. Run stack B headless, GPU or software GL; roster per job: navigate/clean.
2. Critical: `drive_path` (collision_monitor/velocity_smoother output) - honest only in-node or as "cmd_vel arrives at 20 Hz while a goal is active";
   `controller_server` - /cmd_vel_nav 5 Hz, only during goal: needs job-state gating; `lidar_bridge` - /scan 5 Hz (period 0.2 s > 0.15 s window,
   needs roster max_age about 0.5 s); `localization` - slam_toolbox /pose 1.6 Hz moving (or /tf map->odom stamp), AMCL /amcl_pose silent when idle:
   external watcher on /tf is too weak, prefer a map->odom age check; `odom` - /odom 50 Hz.
3. Advisory: local/global costmap (published_footprint 5/2 Hz), planner (event driven), bt_navigator, behavior_server, tof, ranges, imu.
4. Heartbeat source: an external topic-activity watcher (new node, subscribes best-effort, beats only when a message was received within the
   component's window, stamps with the ROS clock) is honest for /scan, /odom, /tf, /cmd_vel*, costmap footprints. It cannot tell the controller
   from silent-because-idle, nor catch a wedged-but-publishing node. In-node instrumentation needed for the controller and planner (patch Nav2
   or wrap: no hook exists today); behavior_server, map_server, bt_navigator have no periodic output at all.
5. Hook points: roster publisher started by the task launch (a small launch include next to nav.launch.py / coverage_regression.launch.py,
   latched, per-component max_age_sec); health monitor + stand-in launched with it; nothing in the Nav2 nodes changes.
6. cmd_vel gate: gate the last hop. All writers end at /cmd_vel, whose only subscriber is ros_gz_bridge (verified 1 subscriber). Many task
   nodes publish /cmd_vel directly, bypassing smoother and collision monitor, so gating upstream is not enough. Stand-in: subscribe /cmd_vel
   (remap cmd_vel_in:=/cmd_vel), publish /cmd_vel_gated, and make the bridge listen on /cmd_vel_gated. `parameter_bridge ... -r cmd_vel:=cmd_vel_gated`
   moved its subscription to /cmd_vel_gated (verified with a lone bridge, no Gazebo); full effect on world.launch.py's bridge UNVERIFIED
   (that launch has no remap hook, so a copy or a second bridge is needed; on hardware the gate sits before the serial MCU link).
7. Honesty limit: an external watcher proves output exists and is fresh, not that it is correct. A frozen controller that keeps re-publishing
   the last command, or a localization stuck on a wrong pose, passes. State this in the results.
8. Timing: contract draft says 20 Hz beats and 0.15 s freshness; real sources run at 5 Hz (scan, controller) and 1-2 Hz (pose, costmaps), so
   per-component max_age_sec must be 0.5-1.5 s there; the RFC chain "work period < max_age < MCU soft-stop" then breaks for those components.
   The 0.15 s limit stays realistic only for the drive path.

## 7. Risks, blockers, cost

- Nav2 is one process: "kill the controller" is not possible by PID. Options: lifecycle deactivate/shutdown through the service, SIGSTOP/kill of
  the whole container (takes down all Nav2), or launching controller_server as its own process (Nav2 launches without composition,
  stack A style). Lidar bridge and gz are separate processes (bridge killable, gz sim too).
- Controller/pose topics go quiet when idle: false trips between goals unless the roster follows job state.
- Startup: nodes join over about 40 s (gz, then AMCL/slam, then Nav2 active); roster must be published after Nav2 is active, or arming
  must tolerate the join (RFC note about nodes joining stalling).
- Stale processes between runs (section 1) break graph checks.
- RTF: software GL holds 1.0 with all sensors; wall-clock watchdogs false-trip below 1.0 (FINDINGS). Stay on the ROS clock.
- Machine time per soak hour: stack B ~ 1 h wall at RTF 1.0 = about 1.2 cores (gz 0.8 + nav2 0.4 + bridge 0.1) in software, about 0.9 on GPU;
  plus ~40 s startup per run and per-fault-trial restarts. A 1-hour sim soak = 1 hour wall at RTF 1; unlocked about 20-22 min per sim hour
  (single 15 s sample; stability over hours UNVERIFIED). Coverage job of 313 waypoints is about 15-18 min wall at RTF 1.
- Not measured: dock, mapping, fault injection, runs over 15 min, memory growth, determinism.
