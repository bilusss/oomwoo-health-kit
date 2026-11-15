#!/usr/bin/env python3
"""Live-graph facts for the watcher configs (M7, run inside an ohk-sim-<i> with the stack up).

  python3 graph_facts.py [--out graph.json] [topic ...]

For each topic: type, every publisher (node, QoS reliability/durability/depth) and every subscription. With no topic arguments the
topics that matter for the health loop are listed. Also lists nodes and their process ids' command names (from /proc by node
name is not possible for composed nodes, so only the node list is given). A watcher config that watches a topic with the wrong
writer set is a fail-open defect; this is the check that precedes writing one.
"""

import argparse
import json
import time

import rclpy
from rclpy.node import Node

DEFAULT = ["/cmd_vel", "/cmd_vel_nav", "/cmd_vel_smoothed", "/cmd_vel_gated", "/cmd_vel_teleop", "/scan", "/odom", "/tf",
           "/amcl_pose", "/pose", "/particle_cloud", "/local_costmap/published_footprint", "/global_costmap/published_footprint",
           "/behavior_server/transition_event", "/plan", "/dock_drive/state", "/coverage_planner/cleaning_active",
           "/coverage_meter/ratio", "/ground_truth/pose"]


def endpoint(i):
    q = i.qos_profile
    return {"node": (i.node_namespace.rstrip("/") + "/" + i.node_name), "type": i.topic_type,
            "reliability": q.reliability.name, "durability": q.durability.name, "depth": q.depth}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=None)
    ap.add_argument("topics", nargs="*")
    a = ap.parse_args()
    rclpy.init()
    node = Node("ohk_graph_facts")
    time.sleep(8.0)  # discovery (3 s missed the bridge endpoints in a first try)
    out = {"nodes": sorted((ns.rstrip("/") + "/" + n) for n, ns in node.get_node_names_and_namespaces()), "topics": {}}
    for t in a.topics or DEFAULT:
        pubs = [endpoint(i) for i in node.get_publishers_info_by_topic(t)]
        subs = [endpoint(i) for i in node.get_subscriptions_info_by_topic(t)]
        out["topics"][t] = {"publishers": pubs, "subscriptions": subs}
    print(json.dumps(out, indent=1))
    if a.out:
        json.dump(out, open(a.out, "w"), indent=1)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
