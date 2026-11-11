"""Diagnostic: per-message timeline around component restarts / non-roster node starts.

Runs inside ohk-dev (ROS sourced). One condition per invocation; writes one JSON line per cycle to --out.
Probe records receipt times (time.monotonic) of: component heartbeats (with their stamp), aggregate mcu_heartbeat
(with the aggregator's tick stamp), stack states, stand-in states.  A "flap" = a stand-in soft_stop transition that
happens after the stand-in has re-armed (RUN) following the restart (or, for non-roster events, any soft stop).

  python3 restart_diag.py --tag base --mode roster_kill --cycles 100
modes: roster_kill (SIGKILL critical, restart), roster_int (SIGINT critical, restart), none (control, no event),
       extra_silent (rclpy node with no pubs, start then SIGKILL), extra_beat (fake_component 'extra' not in roster),
       extra_silent_int
"""
import argparse, json, os, signal, subprocess, sys, time, threading

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
import ohk_bench as b  # noqa: E402
import soak  # noqa: E402  (clean_shm, shm_free)

HERE = os.path.dirname(os.path.abspath(__file__))
SILENT = ("import rclpy,time\nfrom rclpy.node import Node\nrclpy.init()\nn=Node('diag_silent')\n"
          "while True: rclpy.spin_once(n, timeout_sec=0.1)\n")


def make_probe(cmd=True):
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from std_msgs.msg import String
    from oomwoo_health_kit.qos import latched_state_qos, live_heartbeat_qos, mcu_heartbeat_qos

    class P(Node):
        def __init__(self):
            super().__init__("diag_probe")
            self.lock = threading.Lock()
            self.ev = []  # (t_mono, kind, data)
            self.last_state = None
            self.last_cmd = None
            self.pub = self.create_publisher(Twist, "cmd_vel_in", 10)
            self.create_subscription(Twist, "cmd_vel", self.on_cmd, 10)
            self.create_subscription(String, "oomwoo/health/mcu_state", self.on_state, latched_state_qos())
            self.create_subscription(String, "oomwoo/health/stack", self.on_stack, latched_state_qos())
            self.create_subscription(String, "oomwoo/health/mcu_heartbeat", self.on_mcu, mcu_heartbeat_qos())
            self.create_subscription(String, "oomwoo/health/component", self.on_comp, live_heartbeat_qos())
            if cmd:
                self.create_timer(0.05, self.tick)

        def add(self, k, d):
            with self.lock:
                self.ev.append((time.monotonic(), k, d))

        def tick(self):
            m = Twist(); m.linear.x = 0.2; self.pub.publish(m)

        def on_cmd(self, m):
            nz = m.linear.x != 0.0
            self.last_cmd = (time.monotonic(), nz)
            self.add("cmd", nz)

        def on_state(self, m):
            s = json.loads(m.data)
            self.last_state = s["state"]
            self.add("state", {"s": s["state"], "reason": s["reason"], "stamp": s["stamp"], "soft": s["soft_stops"]})

        def on_stack(self, m):
            s = json.loads(m.data)
            bl = sorted(s["stale_critical"] + s["missing_critical"] + s["unhealthy_critical"])
            self.add("stack", {"s": s["state"], "bl": bl, "stale": s["stale_critical"], "miss": s["missing_critical"]})

        def on_mcu(self, m):
            s = json.loads(m.data)
            self.add("mcu", {"seq": s["sequence"], "stamp": s["stamp_sec"], "wall": time.time()})

        def on_comp(self, m):
            s = json.loads(m.data)
            self.add("comp", {"id": s["component_id"], "seq": s["sequence"], "stamp": s["stamp_sec"], "wall": time.time()})

        def take(self, t0):
            with self.lock:
                return [e for e in self.ev if e[0] >= t0]

        def steady(self):
            c = self.last_cmd
            return self.last_state == "run" and c is not None and c[1] and time.monotonic() - c[0] < 0.1

    return P()


def run(a):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    soak.clean_shm()
    dom = a.domain
    os.environ["ROS_DOMAIN_ID"] = str(dom)
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    if a.udp:
        os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = os.path.join(os.path.dirname(HERE), "profiles", "fastdds_udp_only.xml")
    loads = []
    for _ in range(a.load):
        loads.append(subprocess.Popen([sys.executable, "-c", "while True: pass"], start_new_session=True,
                                      preexec_fn=b.die_with_parent))
    procs = b.Procs(dom)
    if a.sleep_trace:  # LD_PRELOAD shim (observation only) for the ROS child processes, not for the probe
        os.makedirs(a.sleep_trace, exist_ok=True)
        procs.env["LD_PRELOAD"] = "/tmp/sleep_trace.so"
        procs.env["SLEEP_TRACE_DIR"] = a.sleep_trace
    out = open(a.out, "a")
    try:
        b.kit_nodes(procs, a.max_age, a.rate,
                    mcu_extra=("-p", f"soft_stop_timeout_s:={a.soft}", "-p", f"rearm_count:={a.rearm}"))
        rclpy.init()
        p = make_probe()
        ex = SingleThreadedExecutor(); ex.add_node(p)
        threading.Thread(target=ex.spin, daemon=True).start()
        if not b.wait_for(lambda: p.steady(), 40):
            raise SystemExit("stack did not arm")
        time.sleep(a.warmup)
        victims = ["critical_a", "critical_b"]
        done = skipped = 0
        t_begin = time.monotonic()
        while done < a.cycles and skipped < 30:
            time.sleep(1.0)
            b.wait_for(p.steady, 10)
            if not p.steady():
                skipped += 1
                continue
            v = victims[done % 2]
            t0 = time.monotonic()
            rec = {"i": done, "victim": v, "mode": a.mode, "t0": t0,
                   "pids_before": {n: pr.pid for n, pr in procs.by_name.items()}}
            extra = None
            if a.mode in ("roster_kill", "roster_int"):
                procs.kill(v, signal.SIGKILL if a.mode == "roster_kill" else signal.SIGINT)
                b.wait_for(lambda: any(k == "cmd" and not d for t, k, d in p.take(t0)), 3.0)
                b.wait_for(lambda: p.last_state != "run", 1.0)
                if a.mode == "roster_int":
                    procs.by_name[v].wait(5)
                tr = time.monotonic()
                b.start_component(procs, v, a.rate)
                rec["t_restart"] = tr
                ok = b.wait_for(lambda: p.steady(), 30)
                rec["rearmed"] = ok
                rec["t_rearm"] = time.monotonic()
                time.sleep(a.tail)
            elif a.mode == "none":
                time.sleep(a.tail + 2)
            else:
                if a.mode in ("extra_silent", "extra_silent_int"):
                    argv = [sys.executable, "-c", SILENT]
                else:
                    argv = [b.exe("oomwoo_health_kit", "fake_component_node"), "--ros-args", "-r", "__node:=extra",
                            "-p", "component_id:=extra", "-p", f"rate_hz:={float(a.rate)}"]
                procs.start("extra", argv)
                rec["t_restart"] = time.monotonic()
                time.sleep(a.extra_up)
                procs.kill("extra", signal.SIGKILL if not a.mode.endswith("_int") else signal.SIGINT)
                rec["t_extra_kill"] = time.monotonic()
                time.sleep(a.tail)
            t1 = time.monotonic()
            rec["pids_after"] = {n: pr.pid for n, pr in procs.by_name.items()}
            evs = p.take(t0 - 0.5)
            rec["t1"] = t1
            rec["ev"] = [(round(t - t0, 4), k, d) for t, k, d in evs if t <= t1]
            # flaps
            states = [(t, d) for t, k, d in evs if k == "state"]
            soft = [t for t, d in states if d["s"] == "soft_stop" and t >= t0]
            if a.mode.startswith("roster"):
                run_after = [t for t, d in states if d["s"] == "run" and t >= t0 and t > (soft[0] if soft else t0)]
                first_run = run_after[0] if run_after else None
                rec["first_run_after_kill"] = None if first_run is None else round(first_run - t0, 4)
                flaps = [round(t - first_run, 4) for t in soft if first_run is not None and t > first_run]
            else:
                flaps = [round(t - t0, 4) for t in soft]
            rec["flaps"] = flaps
            stk = [(t, d) for t, k, d in evs if k == "stack" and t >= (rec.get("t_rearm") or t0)]
            rec["stack_flaps_post_rearm"] = sum(1 for (_, x), (_, y) in zip(stk, stk[1:])
                                                if x["s"].startswith("healthy") and not y["s"].startswith("healthy"))
            out.write(json.dumps(rec) + "\n"); out.flush()
            done += 1
            if done % 20 == 0:
                print(f"[{a.tag}] {done}/{a.cycles} cycles, {time.monotonic()-t_begin:.0f}s", flush=True)
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass
        procs.stop_all()
        for l in loads:
            try: os.killpg(l.pid, signal.SIGKILL)
            except ProcessLookupError: pass
        soak.clean_shm()
        out.close()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", required=True)
    ap.add_argument("--mode", default="roster_kill")
    ap.add_argument("--cycles", type=int, default=100)
    ap.add_argument("--soft", type=float, default=0.15)
    ap.add_argument("--rearm", type=int, default=3)
    ap.add_argument("--max-age", type=float, default=0.15)
    ap.add_argument("--rate", type=int, default=20)
    ap.add_argument("--load", type=int, default=0)
    ap.add_argument("--sleep-trace", help="dir for sleep_trace.so logs (inside container)")
    ap.add_argument("--udp", action="store_true")
    ap.add_argument("--warmup", type=float, default=5.0)
    ap.add_argument("--tail", type=float, default=3.5)
    ap.add_argument("--extra-up", type=float, default=2.5)
    ap.add_argument("--domain", type=int, default=40)
    ap.add_argument("--out")
    a = ap.parse_args()
    assert a.load <= 12
    a.out = a.out or os.path.join(HERE, "..", "..", "results", "diag", f"{a.tag}.jsonl")
    run(a)
