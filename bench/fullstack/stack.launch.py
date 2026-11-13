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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

HERE = os.path.dirname(os.path.abspath(__file__))
MAP = os.path.join(str(get_package_share_path("oomwoo_gazebo")), "maps", "living_room.yaml")


def generate_launch_description():
    args = [
        DeclareLaunchArgument("map", default_value=MAP),
        DeclareLaunchArgument("x_pose", default_value="-2.0"),
        DeclareLaunchArgument("y_pose", default_value="-0.5"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("gpu", default_value="false"),
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
            "headless": "true", "robot_model": "oomwoo_one", "world": "living_room.world", "use_sim_time": "true",
            "x_pose": LC("x_pose"), "y_pose": LC("y_pose"), "gpu": LC("gpu"),
            "enable_lidar": "true", "enable_imu": "false", "enable_ranges": "false",
            "enable_tof": "false", "enable_cameras": "false",
        }.items(),
    )
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(str(get_package_share_path("oomwoo_bringup")), "launch", "navigation.launch.py")),
        launch_arguments={
            "robot_model": "oomwoo_one", "use_sim_time": "true", "map": LC("map"),
            "x_pose": LC("x_pose"), "y_pose": LC("y_pose"), "yaw": LC("yaw"),
        }.items(),
    )
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
                          "sim_time": LC("health_sim_time")}.items())
    return LaunchDescription(args + [world, nav, *health, fake, watcher])
