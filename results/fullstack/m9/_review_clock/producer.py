"""20 Hz sim-time timer beating via the patch's HealthBeat; --div swaps the stamp conversion to nanoseconds / 1e9."""
import sys, rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import oomwoo_health_beat.ros as hb
cid, div = sys.argv[1], '--div' in sys.argv
rclpy.init()
n = Node('prod_' + cid, parameter_overrides=[Parameter('use_sim_time', value=True), Parameter('health_component_id', value=cid)])
beat = hb.HealthBeat(n)
if div:
    def beat_div(level='ok', detail=''):
        now = n.get_clock().now().nanoseconds / 1e9
        beat._pub.publish(hb.String(data=hb.to_json(beat._builder.build(now, level, detail))))
    beat.beat = beat_div
n.create_timer(0.05, lambda: beat.beat())
rclpy.spin(n)
