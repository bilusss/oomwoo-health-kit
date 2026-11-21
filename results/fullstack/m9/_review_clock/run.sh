#!/bin/bash
# $1 = seconds, $2 = number of busy-loop load processes
set -e
source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash
export PYTHONPATH=/beat:$PYTHONPATH ROS_DOMAIN_ID=87 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
cd /exp
for i in $(seq 1 ${2:-0}); do python3 -c 'while True: pass' & done
python3 clock_pub.py & sleep 2
ros2 run oomwoo_health_monitor health_monitor_node --ros-args -p use_sim_time:=true > /tmp/agg.log 2>&1 &
python3 producer.py p_mul & python3 producer.py p_div --div &
sleep 3
python3 roster_and_watch.py p_mul,p_div "$1"
pkill -f clock_pub.py; pkill -f producer.py; pkill -f health_monitor_node; pkill -f 'while True' || true
