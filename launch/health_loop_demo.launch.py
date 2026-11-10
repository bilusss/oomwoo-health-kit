"""Aggregator + MCU stand-in + roster + 2 critical and 1 advisory fake components."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    max_age = LaunchConfiguration("max_age_sec")
    critical = ["critical_a", "critical_b"]
    advisory = ["advisory_a"]
    comps = [
        Node(
            package="oomwoo_health_kit",
            executable="fake_component_node",
            name=f"fake_{cid}",
            parameters=[{"component_id": cid, "rate_hz": 20.0}],
        )
        for cid in critical + advisory
    ]
    return LaunchDescription(
        [
            DeclareLaunchArgument("max_age_sec", default_value="0.15"),
            DeclareLaunchArgument("clock_source", default_value="steady"),
            Node(package="oomwoo_health_monitor", executable="health_monitor_node", name="health_monitor"),
            Node(
                package="oomwoo_health_kit",
                executable="mcu_standin_node",
                name="mcu_standin",
                parameters=[{"clock_source": ParameterValue(LaunchConfiguration("clock_source"), value_type=str)}],
            ),
            Node(
                package="oomwoo_health_kit",
                executable="roster_publisher_node",
                name="roster_publisher",
                parameters=[
                    {
                        "task_id": "demo",
                        "critical_ids": critical,
                        "advisory_ids": advisory,
                        "max_age_sec": ParameterValue(max_age, value_type=float),
                    }
                ],
            ),
            *comps,
        ]
    )
