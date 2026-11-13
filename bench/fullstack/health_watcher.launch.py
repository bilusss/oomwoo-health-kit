"""Heartbeat sources for health_sources:=watcher: the real-node topic watcher + the static roster (M6c).

Provides the latched roster on oomwoo/health/roster (job_roster, state_source static: config/nav_rosters.yaml) and the component
heartbeats on oomwoo/health/component (topic_heartbeat, config/nav_watch_sim.yaml). Nothing in Nav2, slam_toolbox or the bridge is
touched; the watches observe /scan, /odom, map->odom, /cmd_vel and the costmap footprints (verified table in the config header).

Clock: `sim_time` (= the launch's health_sim_time, default false) applies to the watcher; it must equal the aggregator's clock
because the aggregator compares heartbeat stamps with its own clock. Default is WALL time (see docs/SPEC.md, M6c). With wall time
the config has no header-stamp checks (stamps are sim time). max_age_sec and rate_hz are accepted for interface compatibility with
health_fake.launch.py and ignored: the roster file carries the per-component max_age_sec.
Config files are read from the project source tree (read-only mount), so they need no rebuild.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

CONFIG = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "config")


def generate_launch_description():
    sim = ParameterValue(LC("sim_time"), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument("max_age_sec", default_value="0.15"),
        DeclareLaunchArgument("rate_hz", default_value="20.0"),
        DeclareLaunchArgument("sim_time", default_value="false"),
        DeclareLaunchArgument("watch_config", default_value=os.path.join(CONFIG, "nav_watch_sim.yaml")),
        DeclareLaunchArgument("roster_config", default_value=os.path.join(CONFIG, "nav_rosters.yaml")),
        Node(package="oomwoo_health_kit", executable="job_roster", name="job_roster",
             parameters=[{"config_file": LC("roster_config"), "use_sim_time": sim}]),
        Node(package="oomwoo_health_kit", executable="topic_heartbeat", name="topic_heartbeat",
             parameters=[{"config_file": LC("watch_config"), "use_sim_time": sim}]),
    ])
