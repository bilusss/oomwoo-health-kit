"""Full OOMWOO sim stack with the health loop closed on the last cmd_vel hop (M6b).

  Gazebo (living_room, headless) + bridge  <- cmd_vel_gated <- MCU stand-in <- cmd_vel <- Nav2 collision_monitor
  Nav2 + slam_toolbox localization on the shipped map (oomwoo_bringup navigation.launch.py, unmodified)
  aggregator (xbattlax/health-monitor) -> oomwoo/health/mcu_heartbeat -> stand-in
  heartbeat sources: health_sources:=fake (kit fake_component nodes + roster) | watcher (reserved, see health_watcher.launch.py)

Upstream copy: world_gated.launch.py = oomwoo_gazebo/launch/world.launch.py with the bridge remapped to cmd_vel_gated
(+ a `gpu` switch); everything else is stock. Sensors: lidar only (navigation.yaml uses /scan only; imu, side ranges,
ToF and cameras are off). Other /cmd_vel publishers (docking, wall_clean, ...) are not started by this workload; they
would also end at /cmd_vel and be gated.
"""

import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(os.path.dirname(os.path.dirname(HERE)), "config")
MAP = os.path.join(str(get_package_share_path("oomwoo_gazebo")), "maps", "living_room.yaml")


def generate_launch_description():
    args = [
        DeclareLaunchArgument("map", default_value=MAP),
        DeclareLaunchArgument("x_pose", default_value="-2.0"),
        DeclareLaunchArgument("y_pose", default_value="-0.5"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("gpu", default_value="false"),
        # M7: slam = oomwoo_bringup navigation.launch.py (slam_toolbox localization, M6 default);
        #     amcl = oomwoo_clean nav.launch.py (nav2_bringup bringup_launch.py: map_server + AMCL + full Nav2, auto-seeded AMCL)
        DeclareLaunchArgument("localization", default_value="slam", choices=["slam", "amcl", "none"]),
        DeclareLaunchArgument("world", default_value="living_room.world"),
        # M7 job nodes started with the stack: none | coverage (coverage_planner + coverage_meter, nav2 executor)
        # | coverage_reactive (executor reactive: drives /cmd_vel directly) | dock (ir_beacon_sim + oomwoo_dock; kitchen_dining only)
        DeclareLaunchArgument("watch_config", default_value=os.path.join(CONFIG, "nav_watch_sim.yaml")),
        DeclareLaunchArgument("roster_config", default_value=os.path.join(CONFIG, "nav_rosters.yaml")),
        DeclareLaunchArgument("job", default_value="none", choices=["none", "coverage_support", "dock"]),
        # M9: true = set health_component_id on dock_detector / dock_drive (needs the oomwoo_health_beat overlay, tools/sim.sh overlay-build);
        # the coverage planner is started by the bench worker (jobs.planner_cmd). Needs health_sim_time:=true (stamps are sim time).
        DeclareLaunchArgument("inode_health", default_value="false", choices=["true", "false"]),
        # amcl only: true = unmodified oomwoo_clean nav.launch.py (ONE composed nav2_container); false = the same nav2_bringup
        # bringup_launch.py call with use_composition:=False (one process per Nav2 node, needed for per-node SIGKILL/SIGSTOP)
        DeclareLaunchArgument("composition", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("health_sources", default_value="fake", choices=["fake", "watcher"]),
        DeclareLaunchArgument("max_age_sec", default_value="0.15"),
        DeclareLaunchArgument("rate_hz", default_value="20.0"),
        DeclareLaunchArgument("soft_stop_timeout_s", default_value="0.15"),
        DeclareLaunchArgument("clock_source", default_value="steady", choices=["steady", "ros"]),
        # Gazebo publishes /clock at 1000 Hz; every Python node on sim time spends about 0.2 core on it (measured, 5 nodes
        # about 1 core per instance). The health nodes therefore default to wall time (as they would on the robot).
        DeclareLaunchArgument("health_sim_time", default_value="false", choices=["true", "false"]),
    ]
    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(HERE, "world_gated.launch.py")),
        launch_arguments={
            "headless": "true", "robot_model": "oomwoo_one", "world": LC("world"), "use_sim_time": "true",
            "x_pose": LC("x_pose"), "y_pose": LC("y_pose"), "gpu": LC("gpu"),
            "enable_lidar": "true", "enable_imu": "false", "enable_ranges": "false",
            "enable_tof": "false", "enable_cameras": "false",
        }.items(),
    )
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(str(get_package_share_path("oomwoo_bringup")), "launch", "navigation.launch.py")),
        condition=LaunchConfigurationEquals("localization", "slam"),
        launch_arguments={
            "robot_model": "oomwoo_one", "use_sim_time": "true", "map": LC("map"),
            "x_pose": LC("x_pose"), "y_pose": LC("y_pose"), "yaw": LC("yaw"),
        }.items(),
    )
    # AMCL variant: unmodified oomwoo_clean nav.launch.py (default map = oomwoo_sim_support living_room.yaml, world-aligned).
    # The `map` argument of this launch file is NOT forwarded: the slam default (oomwoo_gazebo map) has a different origin.
    nav_amcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(str(get_package_share_path("oomwoo_clean")), "launch", "nav.launch.py")),
        condition=IfCondition(PythonExpression(["'", LC("localization"), "' == 'amcl' and '", LC("composition"), "' == 'true'"])),
        launch_arguments={
            "robot_model": "oomwoo_one", "use_sim_time": "true",
            "x_pose": LC("x_pose"), "y_pose": LC("y_pose"), "yaw": LC("yaw"),
        }.items(),
    )
    # same as nav.launch.py's amcl branch (bringup_launch.py + initialpose_pub seed) with use_composition:=False; the params file
    # and map are the ones nav.launch.py selects
    # nav2_bringup's non-composed map_server did not take the map argument (it loaded the params file's yaml_filename "map.yaml"; the
    # composed path passes the map as a node parameter), so a copy of oomwoo_one's navigation.yaml with the absolute map is written to /tmp.
    def split_nav(context):
        """Evaluated at launch time and only for localization:=amcl composition:=false (other tasks never read navigation.yaml here)."""
        if context.perform_substitution(LC("localization")) != "amcl" or context.perform_substitution(LC("composition")) != "false":
            return []
        split_params = "/tmp/ohk_navigation_amcl_split.yaml"
        living = os.path.join(str(get_package_share_path("oomwoo_sim_support")), "maps", "living_room.yaml")
        with open(os.path.join(str(get_package_share_path("oomwoo_one")), "config", "navigation.yaml")) as f_in, open(split_params, "w") as f_out:
            txt = f_in.read()
            assert 'yaml_filename: "map.yaml"' in txt, "navigation.yaml changed upstream: the split launch needs a new map substitution"
            f_out.write(txt.replace('yaml_filename: "map.yaml"', 'yaml_filename: "%s"' % living))
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(str(get_package_share_path("nav2_bringup")), "launch", "bringup_launch.py")),
                launch_arguments={"map": living, "use_sim_time": "true", "slam": "False", "use_composition": "False",
                                  "params_file": split_params}.items()),
            Node(package="oomwoo_sim_support", executable="initialpose_pub", name="initialpose_pub", output="screen",
                 parameters=[{"use_sim_time": True, "reseed_after_sec": 1.0, "x": ParameterValue(LC("x_pose"), value_type=float),
                              "y": ParameterValue(LC("y_pose"), value_type=float), "yaw": ParameterValue(LC("yaw"), value_type=float)}]),
        ]
    nav_amcl_split = [OpaqueFunction(function=split_nav)]
    # coverage jobs: same nodes and parameters as oomwoo_sim_support coverage_regression.launch.py (cleaning_radius 0.20,
    # robot_radius 0.18, target 0.90), minus its own sim_bringup (the stack above replaces it).
    cov_common = {"cleaning_radius": 0.20, "coverage_target": 0.90, "use_sim_time": True}

    def coverage_nodes(job):
        cond = LaunchConfigurationEquals("job", job)
        return [
            Node(package="oomwoo_sim_support", executable="coverage_meter", output="screen", condition=cond,
                 parameters=[{**cov_common, "robot_radius": 0.1745}],
                 remappings=[("map", "/map"), ("ground_truth/pose", "/ground_truth/pose"),
                             ("cleaning_active", "/coverage_planner/cleaning_active")]),
            Node(package="oomwoo_sim_support", executable="ground_truth", output="screen", condition=cond,
                 parameters=[{"spawn_x": ParameterValue(LC("x_pose"), value_type=float),
                              "spawn_y": ParameterValue(LC("y_pose"), value_type=float),
                              "spawn_yaw": ParameterValue(LC("yaw"), value_type=float), "use_sim_time": True}],
                 remappings=[("odom", "/odom"), ("~/pose", "/ground_truth/pose")]),
        ]
    # The coverage planner itself is started by the bench worker once the loop is armed (it begins driving at once).
    jobs = coverage_nodes("coverage_support")
    # dock_sim.launch.py's nodes without its Shutdown-on-exit of dock_drive (a SIGKILLed dock_drive must not end the stack);
    # dock_drive: exit_when_done false, auto_start false (the bench enables it per attempt through ~/enable).
    # M9: with inode_health:=true the two dock nodes get health_component_id (otherwise the parameter is not passed at all).
    def dock_nodes(context):
        if context.perform_substitution(LC("job")) != "dock":
            return []
        beat = context.perform_substitution(LC("inode_health")) == "true"
        return [
            Node(package="oomwoo_sim_support", executable="ir_beacon_sim", name="ir_beacon_sim", output="screen",
                 parameters=[{"use_sim_time": True, "dock_x": -2.11, "dock_y": -0.5, "dock_yaw_deg": 180.0}],
                 remappings=[("~/bearing", "/dock_detector/beacon_bearing"), ("~/visible", "/dock_drive/beacon_visible")]),
            Node(package="oomwoo_dock", executable="dock_detector", name="dock_detector", output="screen",
                 parameters=[{"use_sim_time": True, "prior_x": 0.6, "prior_y": 0.0, "prior_yaw_deg": 0.0,
                              **({"health_component_id": "dock_detector"} if beat else {})}]),
            Node(package="oomwoo_dock", executable="dock_drive", name="dock_drive", output="screen",
                 parameters=[{"use_sim_time": True, "auto_start": False, "exit_when_done": False,
                              **({"health_component_id": "dock_drive"} if beat else {})}],
                 remappings=[("~/dock_pose", "/dock_detector/dock_pose")]),
        ]
    jobs += [OpaqueFunction(function=dock_nodes)]
    health = [
        Node(package="oomwoo_health_monitor", executable="health_monitor_node", name="health_monitor",
             parameters=[{"use_sim_time": ParameterValue(LC("health_sim_time"), value_type=bool)}]),
        Node(package="oomwoo_health_kit", executable="mcu_standin_node", name="mcu_standin",
             parameters=[{"clock_source": ParameterValue(LC("clock_source"), value_type=str),
                          "soft_stop_timeout_s": ParameterValue(LC("soft_stop_timeout_s"), value_type=float),
                          "use_sim_time": ParameterValue(LC("health_sim_time"), value_type=bool)}],
             # remaps only, the node is unchanged. Input = last hop of the Nav2 chain, output = what the bridge reads.
             remappings=[("cmd_vel_in", "cmd_vel"), ("cmd_vel", "cmd_vel_gated")]),
    ]
    fake = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(HERE, "health_fake.launch.py")),
        condition=LaunchConfigurationEquals("health_sources", "fake"),
        launch_arguments={"max_age_sec": LC("max_age_sec"), "rate_hz": LC("rate_hz"),
                          "sim_time": LC("health_sim_time")}.items())
    watcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(HERE, "health_watcher.launch.py")),
        condition=LaunchConfigurationEquals("health_sources", "watcher"),
        launch_arguments={"max_age_sec": LC("max_age_sec"), "rate_hz": LC("rate_hz"),
                          "sim_time": LC("health_sim_time"), "watch_config": LC("watch_config"),
                          "roster_config": LC("roster_config")}.items())
    return LaunchDescription(args + [world, nav, nav_amcl, *nav_amcl_split, *jobs, *health, fake, watcher])
