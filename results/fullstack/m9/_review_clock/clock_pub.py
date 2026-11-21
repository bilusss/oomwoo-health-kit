"""Publish /clock in 1 ms steps paced at real time (Gazebo publishes /clock at 1000 Hz)."""
import time, rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
rclpy.init(); n = Node('fake_clock')
pub = n.create_publisher(Clock, '/clock', 10)
t0 = time.monotonic(); k = 0
while rclpy.ok():
    k += 1
    ns = k * 1_000_000
    m = Clock(); m.clock.sec = ns // 1_000_000_000; m.clock.nanosec = ns % 1_000_000_000
    pub.publish(m)
    d = t0 + k * 0.001 - time.monotonic()
    if d > 0: time.sleep(d)
