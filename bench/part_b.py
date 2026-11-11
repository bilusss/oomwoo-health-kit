"""Part B: kit nodes against the OOMWOO Gazebo sim (headless), measured in sim and wall time.

Run inside the container (ROS + /ros_ws sourced), nothing else heavy running:

  tools/dev.sh exec 'source /ros_ws/install/setup.bash && python3 /ws/src/oomwoo_health_kit/bench/part_b.py all'

Sub-commands: topology | trips | stops | all.  Results: results/part_b.json (rewritten after every step).
Gazebo is started with the documented headless launch, a bench world that adds the three
systems empty.world lacks (physics, user commands, scene broadcaster), and every rendering
sensor disabled.
"""

import argparse
import bisect
import json
import math
import os
import signal
import statistics
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ohk_bench as b  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(os.path.dirname(HERE), "results")
OUT = os.path.join(RES, "part_b.json")
WORLD = os.path.join(HERE, "worlds", "empty_systems.world")
DOMAIN = 91
MAX_AGE, RATE = 0.15, 20.0
SPEED = 0.2  # oomwoo_one/config/navigation.yaml FollowPath desired_linear_vel; no cleaning speed found
GZ_CMD = ["ros2", "launch", "oomwoo_gazebo", "world.launch.py", "headless:=true", "robot_model:=oomwoo_one",
          f"world:={WORLD}", "x_pose:=0.0", "y_pose:=0.0", "enable_lidar:=false", "enable_ranges:=false",
          "enable_tof:=false", "enable_cameras:=false", "enable_imu:=false"]

DATA_INIT = {"gazebo_command": " ".join(GZ_CMD), "speed_m_s": SPEED, "max_age_sec": MAX_AGE, "rate_hz": RATE}


DATA = dict(DATA_INIT)
if os.path.exists(OUT):  # keep earlier steps when a sub-command is re-run
    try:
        DATA.update(json.load(open(OUT)))
    except ValueError:
        pass


def clean(lines):
    """Drop ANSI colours and the FastDDS shared-memory noise printed by CLI tools in the container."""
    import re

    out = [re.sub(r"\x1b\[[0-9;]*m", "", ln) for ln in lines]
    return [ln for ln in out if "RTPS_" not in ln]


def save():
    os.makedirs(RES, exist_ok=True)
    json.dump(DATA, open(OUT, "w"), indent=1)


def sh(cmd, timeout=20):
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, env=dict(os.environ))
    return (r.stdout + r.stderr).strip()


class Sim:
    def __init__(self):
        self.proc = None

    def start(self):
        self.log = open("/tmp/gz_bench.log", "w")
        self.proc = subprocess.Popen(GZ_CMD, env=dict(os.environ), start_new_session=True,
                                     stdout=self.log, stderr=subprocess.STDOUT)

    def stop(self):
        if not self.proc:
            return
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            try:
                os.killpg(self.proc.pid, sig)
            except ProcessLookupError:
                break
            try:
                self.proc.wait(8)
                break
            except subprocess.TimeoutExpired:
                continue
        subprocess.run(["pkill", "-9", "-f", "gz sim|ruby.*gz|parameter_bridge|robot_state_publisher"],
                       capture_output=True)


def set_rtf(rtf):
    return sh(["gz", "service", "-s", "/world/default/set_physics", "--reqtype", "gz.msgs.Physics",
               "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req",
               f"max_step_size: 0.001 real_time_factor: {rtf}"])


class Run:
    """One probe (fresh per run) with sim extras: /clock, /odom, /odom_wheel, heartbeat stamps."""

    def __init__(self):
        import rclpy
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import String

        from oomwoo_health_kit.qos import live_heartbeat_qos, mcu_heartbeat_qos

        self.rclpy = rclpy
        self.probe = b.make_probe(cmd_speed=SPEED)
        self.clock = []  # (mono, sim)
        self.odom = {"truth": [], "wheel": []}  # (mono, sim_stamp, x, y)
        self.hb = []  # (mono, stamp_sec)
        self.sim_now = None
        p = self.probe

        def on_clock(m):
            s = m.clock.sec + m.clock.nanosec * 1e-9
            self.sim_now = s
            if not self.clock or s - self.clock[-1][1] >= 0.02:
                self.clock.append((time.monotonic(), s))

        def mk(key):
            def cb(m):
                st = m.header.stamp
                self.odom[key].append((time.monotonic(), st.sec + st.nanosec * 1e-9,
                                       m.pose.pose.position.x, m.pose.pose.position.y))
            return cb

        def on_hb(m):
            self.hb.append((time.monotonic(), json.loads(m.data)["stamp_sec"]))

        p.create_subscription(Clock, "/clock", on_clock, 100)
        p.create_subscription(Odometry, "/odom", mk("truth"), 100)
        p.create_subscription(Odometry, "/odom_wheel", mk("wheel"), 100)
        p.create_subscription(String, "oomwoo/health/mcu_heartbeat", on_hb, mcu_heartbeat_qos())
        self.ex, _ = b.spin_in_thread(p)

    def close(self):
        self.ex.shutdown()
        self.probe.destroy_node()

    def rtf(self, since_mono):
        c = [x for x in self.clock if x[0] >= since_mono]
        return (c[-1][1] - c[0][1]) / (c[-1][0] - c[0][0]) if len(c) > 2 else None


def topology(out):
    """Who publishes / subscribes cmd_vel, from the ROS graph and the Gazebo transport graph."""
    r = Run()
    time.sleep(2)
    pubs = [i.node_name for i in r.probe.get_publishers_info_by_topic("/cmd_vel")]
    subs = [i.node_name for i in r.probe.get_subscriptions_info_by_topic("/cmd_vel")]
    r.close()
    subs = [n for n in subs if n != "bench_probe"]  # the probe observes cmd_vel; it is not part of the robot
    out["ros_cmd_vel_publishers"] = pubs
    out["ros_cmd_vel_subscribers"] = subs
    out["ros_topic_info_cmd_vel"] = clean(sh(["ros2", "topic", "info", "/cmd_vel", "-v"]).splitlines())
    out["gz_topic_info_cmd_vel"] = clean(sh(["gz", "topic", "-i", "-t", "/cmd_vel"]).splitlines())
    out["gazebo_log_diffdrive"] = [ln for ln in open("/tmp/gz_bench.log") if "DiffDrive subscribing" in ln][:2]
    out["gazebo_log_diffdrive"] = clean([ln.strip() for ln in out["gazebo_log_diffdrive"]])
    out["ros_cmd_vel_only_stand_in_publishes"] = pubs == ["mcu_standin"]
    out["ros_cmd_vel_only_bridge_subscribes"] = subs == ["ros_gz_bridge"]


def start_kit(procs, clock_source):
    b.kit_nodes(procs, MAX_AGE, RATE, clock_source=clock_source, sim_time=True)


def settle_stop(r, procs):
    """Command zero through the stand-in and wait for the robot to stand still (before teardown)."""
    from geometry_msgs.msg import Twist

    pub = r.probe.create_publisher(Twist, "cmd_vel_in", 10)
    r.probe.speed = 0.0
    for _ in range(10):
        pub.publish(Twist())
        time.sleep(0.05)
    time.sleep(2.5 / max(0.05, 1.0))


def trips_run(clock_source, target_rtf, sim_window):
    procs = b.Procs(DOMAIN)
    r = Run()
    res = {"clock_source": clock_source, "target_rtf": target_rtf, "sim_window_s": sim_window}
    try:
        start_kit(procs, clock_source)
        if not b.wait_run(r.probe, 90.0):
            res["armed"] = False
            return res
        res["armed"] = True
        w0, s0 = time.monotonic(), r.sim_now
        n0 = len(r.hb)
        time.sleep(0.1)
        while r.sim_now - s0 < sim_window:
            time.sleep(0.2)
        w1, s1 = time.monotonic(), r.sim_now
        cmds, states, stacks, _ = r.probe.snapshot()
        s_in = [s for t, s in states if t >= w0]
        trips_msgs = sum(1 for s in s_in if s.get("from") == "run")
        last = states[-1][1]
        st_in = [s for t, s in stacks if t >= w0]
        agg = sum(1 for a, c in zip(st_in, st_in[1:]) if a["state"] != "fault" and c["state"] == "fault")
        hb = [x for x in r.hb[n0:] if x[0] >= w0]
        wall_gaps = [(y[0] - x[0]) * 1000 for x, y in zip(hb, hb[1:])]
        sim_gaps = [(y[1] - x[1]) * 1000 for x, y in zip(hb, hb[1:])]
        res.update({
            "armed": True,
            "sim_elapsed_s": round(s1 - s0, 2), "wall_elapsed_s": round(w1 - w0, 2),
            "measured_rtf": round((s1 - s0) / (w1 - w0), 3),
            "false_trips": max(trips_msgs, last["soft_stops"], last["resets"]),
            "trips_seen_in_state_msgs": trips_msgs, "mcu_soft_stops_total": last["soft_stops"],
            "mcu_resets_total": last["resets"], "clock_fault": last["clock_fault"],
            "aggregator_fault_entries": agg,
            "zero_cmd_msgs_in_window": sum(1 for t, nz in cmds if t >= w0 and not nz),
            "heartbeats": len(hb),
            "heartbeat_max_gap_wall_ms": round(max(wall_gaps), 1) if wall_gaps else None,
            "heartbeat_max_gap_sim_ms": round(max(sim_gaps), 1) if sim_gaps else None,
            "final_state": last["state"],
        })
    finally:
        try:
            settle_stop(r, procs)
        finally:
            procs.stop_all()
            r.close()
    return res


def interp_xy(samples, t):
    ts = [s[1] for s in samples]
    i = bisect.bisect_right(ts, t)
    if i == 0 or i >= len(samples):
        return None
    a, c = samples[i - 1], samples[i]
    f = (t - a[1]) / (c[1] - a[1]) if c[1] > a[1] else 0.0
    return a[2] + f * (c[2] - a[2]), a[3] + f * (c[3] - a[3])


def stop_metrics(samples, t_kill_sim, kill_mono, t_zero_sim):
    p0 = interp_xy(samples, t_kill_sim)
    tail = [s for s in samples if s[1] >= t_kill_sim + 2.5]
    if p0 is None or len(tail) < 5:
        return None
    pf = (statistics.mean(s[2] for s in tail), statistics.mean(s[3] for s in tail))
    spread = max(math.hypot(s[2] - pf[0], s[3] - pf[1]) for s in tail)
    dist = math.hypot(pf[0] - p0[0], pf[1] - p0[1])
    after = [s for s in samples if s[1] >= t_kill_sim]
    still = next((s for s in after if math.hypot(s[2] - pf[0], s[3] - pf[1]) < 0.0005), None)
    if still is None:
        return None
    pz = interp_xy(samples, t_zero_sim) if t_zero_sim else None
    return {
        "distance_m": round(dist, 4),
        "sim_time_to_standstill_s": round(still[1] - t_kill_sim, 3),
        "wall_time_to_standstill_s": round(still[0] - kill_mono, 3),
        "distance_before_zero_cmd_m": round(math.hypot(pz[0] - p0[0], pz[1] - p0[1]), 4) if pz else None,
        "final_rest_spread_m": round(spread, 5),
    }


def stops(n=5, clock_source="steady"):
    procs = b.Procs(DOMAIN)
    r = Run()
    runs, notes = [], []
    try:
        start_kit(procs, clock_source)
        if not b.wait_run(r.probe, 90.0):
            return {"armed": False}
        for i in range(n):
            # reach steady speed: truth x advancing at ~SPEED for 2 s of sim time
            def cruising():
                s = r.odom["truth"]
                if len(s) < 60 or r.sim_now is None:
                    return False
                a = [x for x in s if x[1] >= r.sim_now - 2.0]
                return len(a) > 40 and abs((a[-1][2] - a[0][2]) / (a[-1][1] - a[0][1]) - SPEED) < 0.01 * SPEED * 5

            if not b.wait_for(cruising, 60.0, 0.1):
                notes.append(f"run {i}: never reached cruising speed")
                break
            time.sleep(0.5)
            victim = "critical_a"
            n_odom = len(r.odom["truth"])
            t_sim = r.sim_now
            t0 = time.monotonic()
            procs.kill(victim, signal.SIGKILL)
            zero_ok = b.wait_for(lambda: any(not nz for t, nz in r.probe.snapshot()[0] if t >= t0), 5.0, 0.005)
            zt = next((t for t, nz in r.probe.snapshot()[0] if t >= t0 and not nz), None)
            # sim time at the first zero Twist: nearest clock sample at/after its arrival
            lat_ms = round((zt - t0) * 1000, 1) if zt else None
            # wait for the rest of the stop to be recorded
            b.wait_for(lambda: r.sim_now >= t_sim + 3.2, 60.0, 0.1)
            time.sleep(0.5)
            # sim time at the first zero Twist: first /clock sample received at/after its arrival
            zsim = next((s for m, s in r.clock if m >= zt), None) if zt else None
            m_t = stop_metrics(list(r.odom["truth"]), t_sim, t0, zsim)
            m_w = stop_metrics(list(r.odom["wheel"]), t_sim, t0, zsim)
            runs.append({"run": i + 1, "kill_to_zero_cmd_wall_ms": lat_ms,
                         "truth": m_t, "wheel": m_w, "rtf_during": r.rtf(t0 - 1.0)})
            print(json.dumps(runs[-1]), flush=True)
            # restart victim, wait for re-arm
            b.start_component(procs, victim, RATE, sim_time=True)
            if not b.wait_run(r.probe, 40.0):
                notes.append(f"run {i}: did not re-arm")
                break
            if not zero_ok:
                notes.append(f"run {i}: no zero Twist within 5 s")
        return {"armed": True, "runs": runs, "notes": notes, "clock_source": clock_source}
    finally:
        try:
            settle_stop(r, procs)
        finally:
            procs.stop_all()
            r.close()


def summ(xs):
    xs = [x for x in xs if x is not None]
    return {"n": len(xs), "min": min(xs), "median": statistics.median(xs), "max": max(xs)} if xs else {"n": 0}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["topology", "trips", "stops", "all", "gz_only", "topo_stops"])
    ap.add_argument("--rtfs", default="1.0,0.5,0.25,0.1")
    ap.add_argument("--sim-window", type=float, default=60.0)
    ap.add_argument("--slow-window", type=float, default=30.0, help="sim seconds for target RTF <= 0.1")
    a = ap.parse_args()
    import rclpy

    os.environ["ROS_DOMAIN_ID"] = str(DOMAIN)
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    sim = Sim()
    rclpy.init()
    try:
        t0 = time.monotonic()
        sim.start()
        probe_run = Run()
        ok = b.wait_for(lambda: probe_run.sim_now is not None and len(probe_run.odom["truth"]) > 10
                        and len(probe_run.probe.get_subscriptions_info_by_topic("/cmd_vel")) > 0, 240.0, 0.5)
        DATA["gazebo_ready"] = ok
        DATA["gazebo_startup_wall_s"] = round(time.monotonic() - t0, 1)
        if not ok:
            DATA["failure"] = "gazebo did not become ready in 240 s; see /tmp/gz_bench.log"
            DATA["gazebo_log_tail"] = open("/tmp/gz_bench.log").read().splitlines()[-30:]
            save()
            return
        # idle RTF with no kit nodes
        w = time.monotonic()
        time.sleep(15)
        DATA["idle_rtf_15s"] = round(probe_run.rtf(w), 3)
        DATA["idle_rtf_note"] = "sim time / wall time from /clock samples, Gazebo + bridge only, 5 sensors disabled"
        probe_run.close()
        save()
        if a.cmd == "gz_only":
            return
        if a.cmd in ("topology", "all", "topo_stops"):
            topo = {}
            procs = b.Procs(DOMAIN)
            try:
                start_kit(procs, "steady")
                time.sleep(5)
                topology(topo)
            finally:
                procs.stop_all()
            DATA["topology"] = topo
            save()
        if a.cmd in ("trips", "all"):
            DATA["trips"] = []
            for rtf in [float(x) for x in a.rtfs.split(",")]:
                if rtf < 1.0:
                    DATA.setdefault("set_physics_replies", []).append({"rtf": rtf, "reply": set_rtf(rtf)})
                    time.sleep(3)
                win = a.slow_window if rtf <= 0.1 else a.sim_window
                for cs in ("steady", "ros"):
                    print(f"trips rtf={rtf} clock={cs}", flush=True)
                    res = trips_run(cs, rtf, win)
                    print(json.dumps(res), flush=True)
                    DATA["trips"].append(res)
                    save()
            set_rtf(1.0)
            time.sleep(3)
        if a.cmd in ("stops", "all", "topo_stops"):
            st = stops(5)
            runs = st.get("runs", [])
            st["summary"] = {
                "truth_distance_m": summ([x["truth"]["distance_m"] for x in runs if x["truth"]]),
                "truth_sim_time_to_standstill_s": summ([x["truth"]["sim_time_to_standstill_s"] for x in runs if x["truth"]]),
                "truth_wall_time_to_standstill_s": summ([x["truth"]["wall_time_to_standstill_s"] for x in runs if x["truth"]]),
                "wheel_distance_m": summ([x["wheel"]["distance_m"] for x in runs if x["wheel"]]),
                "kill_to_zero_cmd_wall_ms": summ([x["kill_to_zero_cmd_wall_ms"] for x in runs]),
            }
            DATA["stops"] = st
            save()
    finally:
        sim.stop()
        if rclpy.ok():
            rclpy.shutdown()
        save()


if __name__ == "__main__":
    main()
