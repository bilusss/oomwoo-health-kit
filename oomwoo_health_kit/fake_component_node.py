"""Test component: heartbeats only after each completed work cycle.

Fault injection: SIGUSR1 makes ``work_cycle`` block forever while the process and the
executor thread stay alive.  With ``beat_from_timer:=true`` the heartbeat comes from an
independent timer instead of the work path (the anti-pattern the RFC warns about), so a
stalled work loop is no longer visible to the aggregator.
"""

import signal
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from oomwoo_health_kit.ros_heartbeat import RosHeartbeat


class FakeComponent(Node):
    def __init__(self) -> None:
        super().__init__("fake_component")
        self.declare_parameter("component_id", "fake")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("beat_from_timer", False)
        self._id = self.get_parameter("component_id").value
        self._period = 1.0 / float(self.get_parameter("rate_hz").value)
        self._hb = RosHeartbeat(self, self._id)
        self._stalled = threading.Event()
        signal.signal(signal.SIGUSR1, lambda *_: self._stalled.set())
        if self.get_parameter("beat_from_timer").value:
            self.create_timer(self._period, self._timer_beat)

    def _timer_beat(self) -> None:
        self._hb.beat("ok", "stalled" if self._stalled.is_set() else "")

    def work_cycle(self) -> None:
        """Stand-in for real work; blocks forever once stalled."""
        while self._stalled.is_set() and rclpy.ok():
            time.sleep(0.1)

    def run(self) -> None:
        # Work loop on the main thread, executor (timers, params) on its own thread.
        ex = SingleThreadedExecutor()
        ex.add_node(self)
        threading.Thread(target=ex.spin, daemon=True).start()
        timer_beats = self.get_parameter("beat_from_timer").value
        next_t = time.monotonic()
        while rclpy.ok():
            self.work_cycle()
            if not timer_beats:
                self._hb.beat("ok")
            next_t += self._period
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeComponent()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
