"""Stand-in with clock_source=ros: a frozen /clock must stop the robot (no Gazebo needed)."""

import threading
import time

import launch
import launch_testing.actions
import ohk_harness as h
import rclpy
from launch_ros.actions import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


def generate_test_description():
    h.ACTIONS.clear()
    h.ACTIONS["mcu_standin"] = Node(
        package="oomwoo_health_kit",
        executable="mcu_standin_node",
        name="mcu_standin",
        parameters=[{"clock_source": "ros", "use_sim_time": True}],
    )
    return launch.LaunchDescription([h.ACTIONS["mcu_standin"], launch_testing.actions.ReadyToTest()])


class TestFrozenClock(h.OhkCase):
    def test_frozen_clock_forces_zero_and_is_reported(self):
        clock_pub = self.probe.create_publisher(Clock, "/clock", 10)
        hb_pub = self.probe.create_publisher(String, "oomwoo/health/mcu_heartbeat", h.mcu_heartbeat_qos())
        state = {"t": 1000.0, "run": True}

        def tick():
            if state["run"]:
                state["t"] += 0.01
                m = Clock()
                m.clock.sec = int(state["t"])
                m.clock.nanosec = int((state["t"] % 1) * 1e9)
                clock_pub.publish(m)

        def beat():
            hb_pub.publish(String(data="{}"))

        t1 = self.probe.create_timer(0.01, tick)
        t2 = self.probe.create_timer(0.05, beat)
        self.wait_run()
        state["run"] = False  # clock freezes; heartbeats and commands keep coming
        t0 = time.monotonic()
        self.wait_for(lambda: any(not nz for _, nz in self.cmds_since(t0)), 2.0, "zero cmd_vel")
        lat = next(t for t, nz in self.cmds_since(t0) if not nz) - t0
        self.wait_for(
            lambda: any(s["clock_fault"] for _, s in self.states_since(t0)), 1.0, "clock_fault in mcu_state"
        )
        time.sleep(0.5)
        self.assertTrue(all(not nz for _, nz in self.cmds_since(t0 + lat)))
        self.record("j_frozen_clock", latency_ms=lat * 1000)
        self.probe.destroy_timer(t1)
        self.probe.destroy_timer(t2)
