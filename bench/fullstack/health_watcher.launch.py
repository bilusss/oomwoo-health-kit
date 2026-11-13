"""Placeholder for health_sources:=watcher: the real-node topic watcher (other agent) is not available yet.

When it exists, replace the body with its launch: it must publish the latched roster on oomwoo/health/roster and component
heartbeats on oomwoo/health/component, and accept the arguments max_age_sec and rate_hz below.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, Shutdown


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("max_age_sec", default_value="0.15"),
        DeclareLaunchArgument("rate_hz", default_value="20.0"),
        DeclareLaunchArgument("sim_time", default_value="false"),
        LogInfo(msg="health_sources:=watcher is not implemented yet (real-node watcher pending)"),
        Shutdown(reason="health_watcher.launch.py is a placeholder"),
    ])
