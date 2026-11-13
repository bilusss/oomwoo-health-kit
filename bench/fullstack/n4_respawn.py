#!/usr/bin/env python3
"""N4: how long does the single-writer guard keep a component down after SIGKILL + respawn of its publisher? (M6c, run inside an
ohk-sim-<i> whose stack is up with health_sources:=watcher; robot idle.)

  python3 n4_respawn.py [--target parameter_bridge] [--out n4.json]

Kills the bridge (publisher of /scan and /odom), starts an identical one at once (same argv and environment), and records for
every second: the publisher count of /scan, which components heartbeat, the aggregator state, the stand-in state. The dead
participant stays in discovery until its lease expires, so the count is 2 and the watcher (max_publishers 1) refuses to beat
for lidar / odom / drive_supervision until then. Output: seconds from the kill to the first heartbeat of each component after
the respawn, and to the stand-in RUN again.
"""

import argparse
import json
import os
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from oomwoo_health_kit.qos import latched_state_qos, live_heartbeat_qos


def find(fragment):
    for pid in filter(str.isdigit, os.listdir("/proc")):
        try:
            argv = open(f"/proc/{pid}/cmdline", "rb").read().split(b"\0")
        except OSError:
            continue
        if argv and fragment.encode() in argv[0] and int(pid) != os.getpid():
            return int(pid)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", default="/parameter_bridge")
    ap.add_argument("--out", default=None)
    ap.add_argument("--observe", type=float, default=60.0)
    a = ap.parse_args()
    pid = find(a.target)
    argv = [x.decode() for x in open(f"/proc/{pid}/cmdline", "rb").read().split(b"\0") if x]
    env = dict(x.split(b"=", 1) for x in open(f"/proc/{pid}/environ", "rb").read().split(b"\0") if b"=" in x)
    env = {k.decode(): v.decode() for k, v in env.items()}
    rclpy.init()
    node = Node("ohk_n4")
    lock = threading.Lock()
    beats, states, stacks = [], [], []
    node.create_subscription(String, "oomwoo/health/component",
                             lambda m: (lock.acquire(), beats.append((time.monotonic(), json.loads(m.data)["component_id"])), lock.release()),
                             live_heartbeat_qos())
    node.create_subscription(String, "oomwoo/health/mcu_state",
                             lambda m: states.append((time.monotonic(), json.loads(m.data)["state"])), latched_state_qos())
    node.create_subscription(String, "oomwoo/health/stack",
                             lambda m: stacks.append((time.monotonic(), json.loads(m.data))), latched_state_qos())
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    time.sleep(3.0)
    pubs = []
    t_kill = time.monotonic()
    os.kill(pid, signal.SIGKILL)
    new = subprocess.Popen(argv, env=env, start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    t_respawn = time.monotonic()
    while time.monotonic() - t_kill < a.observe:
        pubs.append((round(time.monotonic() - t_kill, 1), len(node.get_publishers_info_by_topic("/scan")),
                     len(node.get_publishers_info_by_topic("/odom"))))
        time.sleep(1.0)
    out = {"killed_pid": pid, "respawn_after_s": round(t_respawn - t_kill, 3), "argv": argv[:3], "new_pid": new.pid}
    with lock:
        for comp in ("lidar", "odom", "drive_supervision", "localization"):
            after = [t for t, c in beats if c == comp and t >= t_kill]
            before = [t for t, c in beats if c == comp and t < t_kill]
            gaps = [round(b - a_, 2) for a_, b in zip(after, after[1:]) if b - a_ > 1.0]
            out[comp] = {"last_beat_before_kill_s": None if not before else round(before[-1] - t_kill, 2),
                         "first_beat_after_kill_s": None if not after else round(after[0] - t_kill, 2),
                         "first_beat_after_gap_s": next((round(b - t_kill, 2) for a_, b in zip(after, after[1:]) if b - a_ > 1.0), None),
                         "gaps_over_1s": gaps[:4]}
    out["standin_states"] = [(round(t - t_kill, 2), s) for t, s in states if t >= t_kill - 0.1]
    last = None
    out["stack_changes"] = []
    for t, s in stacks:
        key = (s["state"], tuple(s["stale_critical"]), tuple(s["missing_critical"]))
        if t >= t_kill - 0.1 and key != last:
            out["stack_changes"].append((round(t - t_kill, 2), s["state"], s["stale_critical"], s["missing_critical"]))
            last = key
    out["publisher_counts_scan_odom_by_s"] = pubs[::2]
    print(json.dumps(out, indent=1))
    if a.out:
        json.dump(out, open(a.out, "w"), indent=1)
    new.kill()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
