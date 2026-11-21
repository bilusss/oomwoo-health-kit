"""Publish a latched roster (5 s max_age) and count, per component, stack evaluations that list it as stale."""
import json, sys, time, collections, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
ids, secs = sys.argv[1].split(','), float(sys.argv[2])
rclpy.init(); n = Node('watch')
lat = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
rp = n.create_publisher(String, '/oomwoo/health/roster', lat)
rp.publish(String(data=json.dumps({'task_id': 't', 'components': [{'component_id': i, 'critical': True, 'max_age_sec': 5.0} for i in ids]})))
c = collections.Counter(); states = collections.Counter(); onsets = collections.Counter(); prev = [set()]
def on(m):
    d = json.loads(m.data); states[d['state']] += 1
    cur = set(d.get('stale_critical') or [])
    for x in cur: c[x] += 1
    for x in cur - prev[0]: onsets[x] += 1
    prev[0] = cur
n.create_subscription(String, '/oomwoo/health/stack', on, lat)
t0 = time.monotonic()
while time.monotonic() - t0 < secs:
    rclpy.spin_once(n, timeout_sec=0.1)
print(json.dumps({'stack_msgs_by_state': dict(states), 'stale_evaluations': dict(c), 'stale_onsets': dict(onsets)}))
