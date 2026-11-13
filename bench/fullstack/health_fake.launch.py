"""Heartbeat sources for health_sources:=fake: one latched roster + kit fake_component nodes (work-path beats).

Interchangeable with health_watcher.launch.py: both provide the roster on oomwoo/health/roster and component heartbeats on
oomwoo/health/component. These fake components do NOT watch the real Nav2/Gazebo nodes; the loop arms and the stand-in gates
cmd_vel, but a Nav2 fault would not be noticed.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

CRITICAL = ["critical_a", "critical_b"]
ADVISORY = ["advisory_a"]


def generate_launch_description():
    comps = [
        Node(package="oomwoo_health_kit", executable="fake_component_node", name=f"fake_{cid}",
             parameters=[{"component_id": cid, "rate_hz": ParameterValue(LC("rate_hz"), value_type=float),
                          "use_sim_time": ParameterValue(LC("sim_time"), value_type=bool)}])
        for cid in CRITICAL + ADVISORY
    ]
    return LaunchDescription([
        DeclareLaunchArgument("max_age_sec", default_value="0.15"),
        DeclareLaunchArgument("rate_hz", default_value="20.0"),
        DeclareLaunchArgument("sim_time", default_value="false"),
        Node(package="oomwoo_health_kit", executable="roster_publisher_node", name="roster_publisher",
             parameters=[{"task_id": "fullstack", "critical_ids": CRITICAL, "advisory_ids": ADVISORY,
                          "max_age_sec": ParameterValue(LC("max_age_sec"), value_type=float),
                          "use_sim_time": ParameterValue(LC("sim_time"), value_type=bool)}]),
        *comps,
    ])
