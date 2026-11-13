#!/usr/bin/env python3
"""Full-stack sessions (M6b): real OOMWOO sim (Gazebo + Nav2 + slam_toolbox localization) with the MCU stand-in gating the last
cmd_vel hop, driven by the NavigateToPose goal loop in workload.py. Runs INSIDE an ohk-sim-<i> container; use tools/sim.sh.

  python3 bench/fullstack/run.py [--duration-sim 300] [--rtf 1.0] [--profile default[,shm_hc60,...]] [--sessions 1]
                                 [--gpu] [--health-sources fake|watcher] [--max-age 0.15] [--quiet-window A/B ...]

One session = start the stack, wait until it is armed and localized, run the goal loop for --duration-sim seconds of SIM time at
the given real-time factor, tear everything down. One JSON line per session is appended (flushed, fsynced) to
<results>/sessions.jsonl; events go to <results>/run.log; <results>/progress.json is rewritten every few seconds; the stack's own
output is in <results>/stack_<session>.log (last session only is kept per index modulo 3).

Time bases (every number says which): *_sim_* = Gazebo /clock; *_wall_* = host monotonic clock. Soft-stop and gap timestamps are
wall clock (time.monotonic offset to local ISO time) with the sim time at the same moment. The stand-in runs on the steady (wall)
clock, so wall is the reference for stand-in decisions; at RTF 1.0 sim and wall agree. Heartbeat gaps are what a SEPARATE probe
subscriber (not the stand-in) sees on oomwoo/health/mcu_heartbeat: a gap there can also be probe-side scheduling delay.

--profile is a comma list cycled per session (default, default_subnet, udp_only, shm_hc60: bench/profiles/*.xml via
FASTRTPS_DEFAULT_PROFILES_FILE). The variable and the discovery range are set in this process's environment before any child is
spawned, so the Gazebo bridge, Nav2 container, slam_toolbox, aggregator, stand-in and the probe all inherit them; the worker
re-reads /proc/<pid>/environ of the key processes to verify it. Fast DDS reads the file once per process, hence a fresh worker
process per session (as soak.py does since M5).

Validity (a session is valid only if all hold): armed + localized within the arm timeout; /clock advanced >= 90% of the requested
sim duration and never ran backwards; robot moved (>= 1 m of odom path and >= 1 goal succeeded); stand-in input and gated output
both carried messages; key processes alive to the end; /dev/shm free >= 64 MiB throughout; the profile's expected shm behaviour
(shm_hc60 shows fastrtps_* files, udp_only none); measured RTF within 5% of requested overall and >= 90% in every 30 s window;
no quiet window started during the session. Anything else is data, not invalidity.

Quiet windows (--quiet-window "2026-11-13T08:00/2026-11-13T22:00", local time; set TZ): no session is started inside a window
or when it could not finish before the next one (the run sleeps until the window ends). Unlike soak.py a running stack is not
paused (a paused Gazebo is not a meaningful session): a session that overruns into a window is aborted and marked invalid.
SIGTERM/SIGINT: partial result is written, stack stopped, exit.
"""

import argparse
import datetime as dt
import glob
import json
import os
import signal
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
BENCH = os.path.dirname(HERE)
KIT_ROOT = os.path.dirname(BENCH)
sys.path.insert(0, BENCH)
sys.path.insert(0, HERE)
import ohk_bench as b  # noqa: E402
import soak as s  # noqa: E402  (profiles, windows, shm helpers; the parent never imports rclpy)

RES = os.environ.get("OHK_RESULTS_DIR") or ("/results" if os.path.isdir("/results") else os.path.join(KIT_ROOT, "results", "fullstack"))
LAUNCH = os.path.join(HERE, "stack.launch.py")
ARM_TIMEOUT_S = 240.0
HB_GAP_S = 0.2
NOMINAL_HZ = 20.0
RTF_WINDOW_S = 30.0
# Regexes (the [x] form keeps pkill from matching its own command line) for everything a session can leave behind.
KILL_RE = ("[g]z sim|[r]uby .*gz|[c]omponent_container|[p]arameter_bridge|[r]obot_state_publisher|[l]ocalization_slam_toolbox|"
           "[h]ealth_monitor_node|[m]cu_standin_node|[r]oster_publisher_node|[f]ake_component_node|[t]opic_heartbeat_node|"
           "[j]ob_roster_node|[b]ench/fullstack/run.py --(worker|recorder)|[r]viz2|[r]os2 launch|[r]os2cli.daemon|[r]os_gz_sim|ros_gz_sim/create|[l]aunch_ros")
# Processes whose death invalidates a session, by /proc comm or a cmdline fragment.
KEY_PROCS = {"gz": "gz sim -s", "nav2_container": "component_container", "bridge": "parameter_bridge",
             "aggregator": "health_monitor_node", "standin": "mcu_standin_node", "slam": "localization_slam_toolbox"}


class Shutdown(BaseException):
    pass


_stopping = False


def _on_signal(signum, _frame):
    global _stopping
    if _stopping:
        return
    _stopping = True
    raise Shutdown(signal.Signals(signum).name)


def iso(t_mono, off):
    return dt.datetime.fromtimestamp(t_mono + off).astimezone().isoformat(timespec="milliseconds")


# --------------------------------------------------------------------------- /proc and cgroup helpers (no ROS)

def pgrep(fragment):
    r = subprocess.run(["pgrep", "-f", fragment], capture_output=True, text=True)
    return [int(x) for x in r.stdout.split() if int(x) != os.getpid()]


def cgroup_cpu_s():
    """CPU seconds used by this container (cgroup v2 cpu.stat), or None."""
    for path in ("/sys/fs/cgroup/cpu.stat",):
        try:
            for line in open(path):
                if line.startswith("usage_usec"):
                    return int(line.split()[1]) / 1e6
        except OSError:
            pass
    return None


def host_cpu_busy_s():
    """(busy, total) CPU seconds over all host CPUs from /proc/stat (host-wide, includes other containers)."""
    f = open("/proc/stat").readline().split()[1:]
    v = [int(x) for x in f]
    idle = v[3] + v[4]
    tck = os.sysconf("SC_CLK_TCK")
    return (sum(v) - idle) / tck, sum(v) / tck


def proc_cpu_by_comm():
    """{comm: cpu seconds} summed over all processes (utime+stime from /proc/<pid>/stat)."""
    tck = os.sysconf("SC_CLK_TCK")
    out = {}
    for pid in filter(str.isdigit, os.listdir("/proc")):
        try:
            raw = open(f"/proc/{pid}/stat").read()
            comm = raw[raw.index("(") + 1: raw.rindex(")")]
            f = raw[raw.rindex(")") + 2:].split()
            out[comm] = out.get(comm, 0.0) + (int(f[11]) + int(f[12])) / tck
        except (OSError, ValueError, IndexError):
            continue
    return out


def key_procs_alive():
    return {name: bool(pgrep(frag)) for name, frag in KEY_PROCS.items()}


def kill_leftovers(log, wait=6.0):
    subprocess.run(["pkill", "-9", "-f", KILL_RE], capture_output=True)
    end = time.monotonic() + wait
    left = ""
    while time.monotonic() < end:
        r = subprocess.run(["pgrep", "-af", KILL_RE], capture_output=True, text=True)
        left = "\n".join(l for l in r.stdout.splitlines() if str(os.getpid()) != l.split()[0])
        if not left:
            return []
        time.sleep(0.5)
    log(f"  LEFTOVER processes after kill -9: {left}")
    return left.splitlines()


def write_json_atomic(path, obj):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(obj, f)
    os.replace(tmp, path)


def gpu_snapshot():
    """nvidia-smi evidence: compute/graphics apps and utilisation. Strings as printed; None if unavailable."""
    def q(*a):
        try:
            return subprocess.run(["nvidia-smi", *a], capture_output=True, text=True, timeout=15).stdout.strip()
        except (OSError, subprocess.TimeoutExpired):
            return None
    return {"apps": q("--query-compute-apps=pid,process_name,used_memory", "--format=csv,noheader"),
            "gpu": q("--query-gpu=utilization.gpu,memory.used,memory.total", "--format=csv,noheader"),
            "pmon": q("pmon", "-c", "1")}


def write_egl_vendor():
    path = "/tmp/10_nvidia.json"
    with open(path, "w") as f:
        json.dump({"file_format_version": "1.0.0", "ICD": {"library_path": "libEGL_nvidia.so.0"}}, f)
    return path


# --------------------------------------------------------------------------- recorder (own process, wall clock only)

def recorder_main(path):
    """Passive observer of the health topics, in a process of its own so that its view is not delayed by the worker's /clock,
    /odom and action traffic. No sim time: every timestamp is time.monotonic() (CLOCK_MONOTONIC, shared across processes).
    Same QoS as the stand-in's own subscription for the heartbeat (reliable, volatile, depth 1)."""
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from std_msgs.msg import String

    from oomwoo_health_kit.qos import latched_state_qos, mcu_heartbeat_qos

    stop = threading.Event()
    rclpy.init()  # rclpy installs its own SIGINT/SIGTERM handlers; ours must come after
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    node = Node("ohk_fullstack_recorder")
    lock = threading.Lock()
    d = {"hb_buckets": {}, "gaps": [], "states": [], "softs": [], "cmd_in": {}, "cmd_out": {}, "stack_n": 0}
    last = {"hb": None}
    ring = []

    def on_hb(_m):
        t = time.monotonic()
        with lock:
            if last["hb"] is not None and t - last["hb"] > HB_GAP_S:
                d["gaps"].append([last["hb"], t])
            last["hb"] = t
            k = str(int(t))
            d["hb_buckets"][k] = d["hb_buckets"].get(k, 0) + 1

    def on_stack(m):
        t = time.monotonic()
        try:
            j = json.loads(m.data)
        except ValueError:
            return
        with lock:
            d["stack_n"] += 1
            ring.append((t, j))
            if len(ring) > 80:
                del ring[:40]

    def on_state(m):
        t = time.monotonic()
        try:
            j = json.loads(m.data)
        except ValueError:
            return
        with lock:
            d["states"].append({"t": t, "state": j.get("state"), "from": j.get("from"), "reason": j.get("reason"),
                                "standin_stamp": j.get("stamp")})
            if j.get("state") == "soft_stop":
                d["softs"].append({"t": t, "standin_stamp": j.get("stamp"), "reason": j.get("reason"),
                                   "ctx": [{"age_wall_s": round(t - ts, 3), **sd} for ts, sd in ring[-4:]],
                                   "last_hb_age_wall_s": None if last["hb"] is None else round(t - last["hb"], 3)})

    def cmd_cb(key):
        def cb(m):
            k = str(int(time.monotonic()))
            with lock:
                c = d[key].setdefault(k, [0, 0])
                c[0] += 1
                if m.linear.x != 0.0 or m.angular.z != 0.0:
                    c[1] += 1
        return cb

    node.create_subscription(String, "oomwoo/health/mcu_heartbeat", on_hb, mcu_heartbeat_qos())
    node.create_subscription(String, "oomwoo/health/stack", on_stack, latched_state_qos())
    node.create_subscription(String, "oomwoo/health/mcu_state", on_state, latched_state_qos())
    node.create_subscription(Twist, "cmd_vel", cmd_cb("cmd_in"), 10)
    node.create_subscription(Twist, "cmd_vel_gated", cmd_cb("cmd_out"), 10)

    def dump():
        with lock:
            write_json_atomic(path, d)

    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    n = 0
    while not stop.wait(1.0):
        n += 1
        if n % 20 == 0:
            dump()
    dump()
    if rclpy.ok():
        rclpy.shutdown()


def _interp(pairs, t):
    """sim time at monotonic time t from (mono, sim) samples; None outside the sampled range."""
    if not pairs or t < pairs[0][0] - 1.0 or t > pairs[-1][0] + 1.0:
        return None
    lo, hi = pairs[0], pairs[-1]
    for a, b_ in zip(pairs, pairs[1:]):
        if a[0] <= t <= b_[0]:
            lo, hi = a, b_
            break
    if hi[0] == lo[0]:
        return round(lo[1], 3)
    return round(lo[1] + (hi[1] - lo[1]) * (t - lo[0]) / (hi[0] - lo[0]), 3)


# --------------------------------------------------------------------------- worker (imports rclpy)

def worker_main(spec_path):
    spec = json.load(open(spec_path))
    import rclpy
    from rclpy.duration import Duration
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from rclpy.time import Time
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    import tf2_ros

    from oomwoo_health_kit.qos import latched_state_qos
    import workload as wl

    stop = threading.Event()
    log = s.Logger(os.path.join(RES, "run.log"))
    off = time.time() - time.monotonic()
    rec_path = spec["result"] + ".recorder"
    recorder = subprocess.Popen([sys.executable, "-u", os.path.abspath(__file__), "--recorder", rec_path], env=dict(os.environ),
                                start_new_session=True, preexec_fn=b.die_with_parent)

    rclpy.init()  # rclpy installs its own SIGINT/SIGTERM handlers; ours must come after
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    # Wall-clock node; sim time comes from one explicit /clock subscription (cheaper than use_sim_time + a TF listener).
    node = Node("ohk_fullstack_probe")
    lock = threading.Lock()
    st = {"clock": 0.0, "last_state": None, "stack_last": None}

    def sim_now():
        return st["clock"]

    def on_clock(m):
        st["clock"] = m.clock.sec + m.clock.nanosec / 1e9

    def on_stack(m):
        try:
            st["stack_last"] = json.loads(m.data)
        except ValueError:
            pass

    def on_state(m):
        try:
            st["last_state"] = json.loads(m.data).get("state")
        except ValueError:
            pass

    node.create_subscription(Clock, "clock", on_clock, QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                                                   history=HistoryPolicy.KEEP_LAST, depth=1))
    node.create_subscription(String, "oomwoo/health/stack", on_stack, latched_state_qos())
    node.create_subscription(String, "oomwoo/health/mcu_state", on_state, latched_state_qos())
    work = wl.Workload(node, sim_now=sim_now, log=lambda m: log("  " + m))
    tfbuf = tf2_ros.Buffer()
    tflist = tf2_ros.TransformListener(tfbuf, node)  # destroyed once localized: /tf is hot
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    res = {"armed": False, "interrupted": False}
    clock_pairs = []  # (monotonic, sim) once per second inside the window
    clock_back = [False]

    def localized():
        try:
            return tfbuf.can_transform("map", "base_footprint", Time(), Duration(seconds=0.0))
        except Exception:
            return False

    from std_srvs.srv import Trigger
    lm_clients = [node.create_client(Trigger, f"/{n}/is_active") for n in ("lifecycle_manager_localization",
                                                                         "lifecycle_manager_navigation")]

    def lifecycle_active():
        """Nav2 lifecycle managers report all their nodes active (the action server exists earlier and rejects goals)."""
        for c in lm_clients:
            if not c.service_is_ready():
                return False
            ev, box = threading.Event(), []
            fut = c.call_async(Trigger.Request())
            fut.add_done_callback(lambda f: (box.append(f.result()), ev.set()))
            if not ev.wait(2.0) or not box or not box[0].success:
                return False
        return True

    def armed():
        return (sim_now() > 0 and lifecycle_active() and st["last_state"] == "run" and (st["stack_last"] or {}).get("emit_mcu_heartbeat") is True
                and work.client.server_is_ready() and localized())

    t_start = time.monotonic()
    while not stop.is_set() and time.monotonic() - t_start < spec["arm_timeout_s"] and not armed():
        time.sleep(0.5)
    res["arm_wall_s"] = round(time.monotonic() - t_start, 1)
    res["armed"] = (not stop.is_set()) and armed()
    if not res["armed"]:
        res["interrupted"] = stop.is_set()
        res["arm_detail"] = {"clock_sim_s": sim_now(), "standin_state": st["last_state"], "stack": st["stack_last"],
                             "nav_server": work.client.server_is_ready(), "map_to_base": localized(),
                             "lifecycle_active": lifecycle_active()}
        finish_recorder(recorder)
        json.dump(res, open(spec["result"], "w"))
        if rclpy.ok():
            rclpy.shutdown()
        return
    try:
        tflist.unregister()
    except Exception:
        pass
    time.sleep(2.0)  # let the first goal start from a settled stack

    # ---- verification of the transport profile in the processes that matter (environment as inherited)
    prof_env = {}
    want = spec["profile_file"] or None
    for name, frag in KEY_PROCS.items():
        pids = pgrep(frag)
        got = "no process"
        for pid in pids[:1]:
            try:
                env = dict(x.split(b"=", 1) for x in open(f"/proc/{pid}/environ", "rb").read().split(b"\0") if b"=" in x)
                got = (env.get(b"FASTRTPS_DEFAULT_PROFILES_FILE", b"").decode() or None)
            except OSError:
                got = "unreadable"
        prof_env[name] = got == want
    res["profile_env_matches"] = prof_env
    res["fastrtps_shm_files_at_start"] = s.shm_file_count()

    # ---- RTF
    if abs(spec["rtf"] - 1.0) > 1e-9:
        r = subprocess.run(["gz", "service", "-s", "/world/default/set_physics", "--reqtype", "gz.msgs.Physics", "--reptype",
                            "gz.msgs.Boolean", "--timeout", "3000", "--req", f"real_time_factor: {spec['rtf']}"],
                           capture_output=True, text=True)
        res["set_rtf_reply"] = (r.stdout + r.stderr).strip()
        log(f"  set real_time_factor {spec['rtf']}: {res['set_rtf_reply']}")

    # ---- the measured window
    cpu0, host0, proc0 = cgroup_cpu_s(), host_cpu_busy_s(), proc_cpu_by_comm()
    d0 = work.odo.distance()
    t0w, t0s = time.monotonic(), sim_now()
    res["window_start_wall"] = iso(t0w, off)
    res["window_start_sim_s"] = round(t0s, 3)
    res["gpu_at_start"] = gpu_snapshot() if spec["gpu"] else None
    gpu_mid_done = [False]
    clock_pairs.append((t0w, t0s))

    def ticker():
        last_sim = t0s
        while not done_evt.is_set():
            t, ss_ = time.monotonic(), sim_now()
            if ss_ < last_sim - 1e-6:
                clock_back[0] = True
            last_sim = ss_
            clock_pairs.append((t, ss_))
            if spec["gpu"] and not gpu_mid_done[0] and ss_ - t0s > spec["duration_sim_s"] / 2:
                gpu_mid_done[0] = True
                res["gpu_mid"] = gpu_snapshot()
                res["fastrtps_shm_files_mid"] = s.shm_file_count()
            ok = sum(1 for r in work.records if r["result"] == "succeeded")
            write_json_atomic(os.path.join(RES, "progress.json"), {
                "phase": "workload", "session": spec["session"], "profile": spec["profile"],
                "sim_elapsed_s": round(ss_ - t0s, 1), "wall_elapsed_s": round(t - t0w, 1), "goals_ok": ok,
                "goals": len(work.records), "distance_m": round(work.odo.distance() - d0, 1), "standin": st["last_state"],
                "updated": dt.datetime.now().astimezone().isoformat(timespec="seconds")})
            done_evt.wait(1.0)

    done_evt = threading.Event()
    threading.Thread(target=ticker, daemon=True).start()
    try:
        work.run(spec["duration_sim_s"], should_stop=stop.is_set)
    finally:
        t1w, t1s = time.monotonic(), sim_now()
        done_evt.set()
        clock_pairs.append((t1w, t1s))
    cpu1, host1, proc1 = cgroup_cpu_s(), host_cpu_busy_s(), proc_cpu_by_comm()
    rec = finish_recorder(recorder, rec_path)
    wall, sim = t1w - t0w, t1s - t0s
    res["interrupted"] = stop.is_set()
    res["window_end_wall"] = iso(t1w, off)
    res["wall_s"] = round(wall, 2)
    res["sim_s"] = round(sim, 2)
    res["rtf"] = round(sim / wall, 4) if wall > 0 else None
    wins = []  # sliding 30 s wall windows, step 10 samples (~10 s)
    for i in range(0, len(clock_pairs), 10):
        j = next((k for k in range(i, len(clock_pairs)) if clock_pairs[k][0] - clock_pairs[i][0] >= RTF_WINDOW_S), None)
        if j is None:
            break
        wins.append((clock_pairs[j][1] - clock_pairs[i][1]) / (clock_pairs[j][0] - clock_pairs[i][0]))
    res["rtf_30s_windows"] = {"n": len(wins), "min": round(min(wins), 3) if wins else None,
                              "mean": round(sum(wins) / len(wins), 3) if wins else None}
    res["clock_went_backwards"] = clock_back[0]
    res["cpu_cores_container"] = round((cpu1 - cpu0) / wall, 3) if cpu0 is not None and wall > 0 else None
    res["cpu_cores_host_busy"] = round((host1[0] - host0[0]) / wall, 3) if wall > 0 else None
    res["cpu_host_logical_cpus"] = os.cpu_count()
    res["loadavg_end"] = open("/proc/loadavg").read().split()[:3]
    top = sorted(((c, (proc1.get(c, 0) - proc0.get(c, 0)) / wall) for c in proc1), key=lambda x: -x[1])[:8]
    res["cpu_cores_by_process"] = {c: round(v, 3) for c, v in top if v > 0.005}
    res["cpu_basis"] = ("container = cgroup usage_usec delta over the window (includes the worker and recorder); host_busy = "
                        "/proc/stat all CPUs (includes other containers); by_process = utime+stime per comm, live processes only")
    res["fastrtps_shm_files_end"] = s.shm_file_count()
    if spec["gpu"]:
        res["gpu_at_end"] = gpu_snapshot()

    recs = work.records
    counted = [r for r in recs if not (r["result"] == "canceled" and r.get("note") == "session ended")]
    res["goals_attempted"] = len(counted)
    res["goals_succeeded"] = sum(1 for r in counted if r["result"] == "succeeded")
    res["goals_by_result"] = {}
    for r in recs:
        res["goals_by_result"][r["result"]] = res["goals_by_result"].get(r["result"], 0) + 1
    res["distance_m"] = round(work.odo.distance() - d0, 3)
    res["distance_basis"] = "path length integrated from /odom (ground-truth odometry) over the window"
    ok = [r for r in counted if r["result"] == "succeeded"]
    res["goal_duration_sim_s"] = ({"min": min(r["duration_sim_s"] for r in ok), "max": max(r["duration_sim_s"] for r in ok),
                                    "mean": round(sum(r["duration_sim_s"] for r in ok) / len(ok), 2)} if ok else None)
    res["goals"] = recs

    # ---- recorder data, restricted to the window; times converted to ISO wall + sim
    if rec is None:
        res["recorder_missing"] = True
    else:
        def ev(t):
            return {"wall": iso(t, off), "sim_s": _interp(clock_pairs, t)}
        inw = lambda t: t0w <= t <= t1w  # noqa: E731
        res["soft_stops"] = [{**ev(e["t"]), "standin_stamp_steady_s": e["standin_stamp"], "reason": e["reason"],
                              "last_heartbeat_age_wall_s": e["last_hb_age_wall_s"],
                              "aggregator_stack_at_event": e["ctx"][-1] if e["ctx"] else None,
                              "aggregator_stack_recent": e["ctx"]} for e in rec["softs"] if inw(e["t"])]
        res["soft_stops_n"] = len(res["soft_stops"])
        res["standin_transitions"] = [{**ev(e["t"]), "state": e["state"], "from": e["from"], "reason": e["reason"]}
                                      for e in rec["states"] if inw(e["t"])]
        gaps = [(a, b_) for a, b_ in rec["gaps"] if inw(b_)]
        res["hb_gaps_over_200ms"] = [{**ev(b_), "gap_wall_ms": round((b_ - a) * 1000, 1)} for a, b_ in gaps]
        res["hb_gaps_n"] = len(gaps)
        res["hb_gaps_max_ms"] = round(max(((b_ - a) * 1000 for a, b_ in gaps), default=0.0), 1)
        res["hb_gaps_basis"] = ("gaps between consecutive oomwoo/health/mcu_heartbeat messages seen by the recorder process "
                                "(same QoS as the stand-in), wall clock; the stand-in's own view is not exposed")
        sec = lambda k: t0w <= float(k) + 0.5 <= t1w  # noqa: E731
        res["hb_msgs"] = sum(v for k_, v in rec["hb_buckets"].items() if sec(k_))
        res["hb_rate_hz_wall"] = round(res["hb_msgs"] / wall, 2) if wall > 0 else None
        res["hb_rate_hz_sim"] = round(res["hb_msgs"] / sim, 2) if sim > 0 else None
        res["hb_nominal_hz"] = NOMINAL_HZ
        for key, name in (("cmd_in", "cmd_vel_in"), ("cmd_out", "cmd_vel_gated")):
            res[name + "_msgs"] = sum(v[0] for k_, v in rec[key].items() if sec(k_))
            res[name + "_nonzero"] = sum(v[1] for k_, v in rec[key].items() if sec(k_))
        res["recorder_stack_msgs"] = rec["stack_n"]
    res["standin_end_state"] = st["last_state"]
    res["last_stack"] = st["stack_last"]
    json.dump(res, open(spec["result"], "w"))
    if rclpy.ok():
        rclpy.shutdown()


def finish_recorder(proc, path=None):
    if proc.poll() is None:
        proc.send_signal(signal.SIGTERM)
        try:
            proc.wait(15)
        except subprocess.TimeoutExpired:
            proc.kill()
    if path and os.path.exists(path):
        try:
            return json.load(open(path))
        except ValueError:
            return None
    return None



# --------------------------------------------------------------------------- parent

def session_record(args, k, profile, log, windows):
    prof_file = s.PROFILES[profile]
    rec = {"session": k, "profile": profile, "profile_file": prof_file, "profile_sha256": s.profile_sha256(profile),
           "discovery_range": s.DISCOVERY_RANGE[profile], "requested_rtf": args.rtf, "requested_duration_sim_s": args.duration_sim,
           "gpu": args.gpu, "health_sources": args.health_sources, "max_age_sec": args.max_age, "label": args.label,
           "domain_id": os.environ.get("ROS_DOMAIN_ID"), "instance": os.environ.get("OHK_INSTANCE"),
           "kit_commit": os.environ.get("OHK_KIT_COMMIT"), "aggregator_commit": os.environ.get("OHK_AGG_COMMIT"),
           "start": dt.datetime.now().astimezone().isoformat(timespec="seconds"), "valid": False,
           "timing_note": "sim = Gazebo /clock, wall = host monotonic; stand-in decides on wall clock (steady)"}
    reasons = []
    spec_path = f"/tmp/ohk_fs_spec_{k}.json"
    res_path = f"/tmp/ohk_fs_res_{k}.json"
    for p in (res_path,):
        if os.path.exists(p):
            os.unlink(p)
    stack = worker = None
    stack_log = open(os.path.join(RES, f"stack_{k % 3}.log"), "w")
    prog = {"session": k, "profile": profile, "phase": "cleanup"}
    off = time.time() - time.monotonic()

    def progress(phase, **kw):
        prog.update(phase=phase, updated=dt.datetime.now().astimezone().isoformat(timespec="seconds"), **kw)
        write_json_atomic(os.path.join(RES, "progress.json"), prog)

    try:
        progress("cleanup")
        kill_leftovers(log)
        s.clean_shm(log)
        if s.shm_free() < s.SHM_MIN_FREE:
            raise s.CellInvalid(f"/dev/shm free {s.shm_free() / 1e6:.1f} MB < 64 MiB before start")
        s.apply_profile(profile)
        os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = s.DISCOVERY_RANGE[profile]
        if args.gpu:
            os.environ["OHK_EGL_VENDOR_FILE"] = write_egl_vendor()
            os.environ.pop("LIBGL_ALWAYS_SOFTWARE", None)
            os.environ.pop("GALLIUM_DRIVER", None)
        rec["env_at_launch"] = {k_: os.environ.get(k_) for k_ in ("ROS_DOMAIN_ID", "ROS_AUTOMATIC_DISCOVERY_RANGE",
                                "FASTRTPS_DEFAULT_PROFILES_FILE", "GZ_PARTITION", "OHK_GPU", "NVIDIA_DRIVER_CAPABILITIES")}
        cmd = ["ros2", "launch", LAUNCH, f"gpu:={'true' if args.gpu else 'false'}", f"health_sources:={args.health_sources}",
               f"max_age_sec:={args.max_age}", f"rate_hz:={args.rate_hz}", f"soft_stop_timeout_s:={args.soft_stop}"]
        log(f"session {k}: profile {profile}, gpu {args.gpu}, duration {args.duration_sim} sim s, rtf {args.rtf}")
        progress("starting stack")
        t_launch = time.monotonic()
        stack = subprocess.Popen(cmd, stdout=stack_log, stderr=subprocess.STDOUT, start_new_session=True,
                                 preexec_fn=b.die_with_parent)
        spec = {"session": k, "profile": profile, "profile_file": prof_file or "", "duration_sim_s": args.duration_sim,
                "rtf": args.rtf, "arm_timeout_s": ARM_TIMEOUT_S, "gpu": args.gpu, "result": res_path}
        json.dump(spec, open(spec_path, "w"))
        worker = subprocess.Popen([sys.executable, "-u", os.path.abspath(__file__), "--worker", spec_path], env=dict(os.environ),
                                  start_new_session=True, preexec_fn=b.die_with_parent)
        wall_budget = ARM_TIMEOUT_S + args.duration_sim / max(args.rtf, 0.05) * 1.3 + 180.0
        shm_min = s.shm_free()
        died = None
        while worker.poll() is None:
            time.sleep(2.0)
            shm_min = min(shm_min, s.shm_free())
            if shm_min < s.SHM_MIN_FREE:
                reasons.append(f"/dev/shm free dropped to {shm_min / 1e6:.1f} MB")
                break
            if stack.poll() is not None:
                reasons.append(f"ros2 launch exited with {stack.returncode}")
                break
            alive = key_procs_alive()
            if not all(alive.values()) and time.monotonic() - t_launch > 30:
                died = [n for n, ok in alive.items() if not ok]
                reasons.append(f"key process died: {died}")
                break
            if time.monotonic() - t_launch > wall_budget:
                reasons.append(f"wall budget {wall_budget:.0f} s exceeded")
                break
            if windows.active_end(s.now()) is not None:
                reasons.append("quiet window started during the session")
                break
        if worker.poll() is None:  # we broke out: ask the worker to stop and keep what it has
            worker.send_signal(signal.SIGTERM)
            try:
                worker.wait(60)
            except subprocess.TimeoutExpired:
                worker.kill()
        rec["shm_free_min_mb"] = round(shm_min / 1e6, 1)
        # facts that must be read while the stack is still up
        rec["key_procs_alive_at_end"] = key_procs_alive()
        rec["fastrtps_shm_files_before_stop"] = s.shm_file_count()
    except s.CellInvalid as e:
        reasons.append(str(e))
    except Shutdown as e:
        reasons.append(f"aborted by {e}")
        rec["aborted"] = True
        if worker and worker.poll() is None:
            worker.send_signal(signal.SIGTERM)
            try:
                worker.wait(60)
            except subprocess.TimeoutExpired:
                worker.kill()
    finally:
        progress("stopping stack")
        stop_stack(stack, log)
        rec["leftover_after_stop"] = kill_leftovers(log)
        s.clean_shm(log)
        stack_log.close()
    res = {}
    if os.path.exists(res_path):
        try:
            res = json.load(open(res_path))
        except ValueError:
            reasons.append("worker result unreadable")
    else:
        reasons.append("worker wrote no result")
    rec.update(res)
    rec["end"] = dt.datetime.now().astimezone().isoformat(timespec="seconds")
    # ---- validity
    checks = {}
    checks["armed"] = bool(res.get("armed"))
    sim, want = res.get("sim_s"), args.duration_sim
    checks["clock_advanced"] = bool(sim is not None and sim >= 0.9 * want and not res.get("clock_went_backwards"))
    checks["robot_moved"] = bool(res.get("distance_m", 0) >= 1.0 and res.get("goals_succeeded", 0) >= 1)
    checks["gating_in_path"] = bool(res.get("cmd_vel_in_msgs", 0) > 0 and res.get("cmd_vel_gated_msgs", 0) > 0)
    kp = rec.get("key_procs_alive_at_end", {})
    checks["key_processes_alive"] = bool(kp) and all(kp.values())
    checks["shm_ok"] = rec.get("shm_free_min_mb", 0) * 1e6 >= s.SHM_MIN_FREE
    exp = s.PROFILE_EXPECTS_SHM[profile]
    n_shm = rec.get("fastrtps_shm_files_before_stop")
    checks["profile_shm_as_expected"] = True if exp is None or n_shm is None else ((n_shm > 0) == exp)
    pe = res.get("profile_env_matches")
    checks["profile_env_in_all_key_processes"] = bool(pe) and all(v is True for v in pe.values()) if prof_file else True
    rtf = res.get("rtf")
    r30 = (res.get("rtf_30s_windows") or {}).get("min")
    checks["rtf_as_requested"] = bool(rtf and abs(rtf - args.rtf) <= 0.05 * args.rtf and (r30 is None or r30 >= 0.9 * args.rtf))
    checks["no_interruption"] = not res.get("interrupted") and not rec.get("aborted")
    checks["no_leftovers"] = not rec.get("leftover_after_stop")
    rec["checks"] = checks
    bad = [n for n, ok in checks.items() if not ok]
    if bad:
        reasons.append("failed checks: " + ", ".join(bad))
    rec["valid"] = not reasons
    if reasons:
        rec["invalid_reason"] = "; ".join(reasons)
    rec["timing_valid"] = bool(abs(args.rtf - 1.0) < 1e-9 and checks.get("rtf_as_requested"))
    log(f"session {k} done: valid={rec['valid']} goals {res.get('goals_succeeded')}/{res.get('goals_attempted')} "
        f"distance {res.get('distance_m')} m soft_stops {res.get('soft_stops_n')} gaps {res.get('hb_gaps_n')} rtf {res.get('rtf')} "
        f"cores {res.get('cpu_cores_container')}" + ("" if rec["valid"] else f" INVALID: {rec['invalid_reason']}"))
    return rec


def stop_stack(stack, log):
    if stack is not None and stack.poll() is None:
        try:
            os.killpg(stack.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            stack.wait(25)
        except subprocess.TimeoutExpired:
            log("  launch did not exit after SIGINT; killing group")
            try:
                os.killpg(stack.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
    time.sleep(0.5)


def parent_main(args):
    os.makedirs(RES, exist_ok=True)
    log = s.Logger(os.path.join(RES, "run.log"))
    windows = s.Windows(args.quiet_window)
    profiles = [p.strip() for p in args.profile.split(",") if p.strip()]
    for p in profiles:
        if p not in s.PROFILES:
            raise SystemExit(f"unknown profile {p!r}; known: {', '.join(s.PROFILES)}")
    if args.gpu and not (os.environ.get("OHK_GPU") == "1"):
        raise SystemExit("--gpu needs a container created with `tools/sim.sh up N --gpu`")
    for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
        signal.signal(sig, _on_signal)
    out_path = os.path.join(RES, "sessions.jsonl")
    log(f"run.py start: profiles {profiles}, sessions {args.sessions or 'endless'}, duration {args.duration_sim} sim s, rtf {args.rtf}, "
        f"gpu {args.gpu}, resources {RES}")
    k = 0
    try:
        while not args.sessions or k < args.sessions:
            est = 120.0 + ARM_TIMEOUT_S * 0.5 + args.duration_sim / max(args.rtf, 0.05) * 1.1 + 60.0
            while True:
                t = s.now()
                end = windows.active_end(t)
                nxt = windows.next_start(t)
                if end is not None:
                    log(f"quiet window until {end.isoformat(timespec='minutes')}: sleeping")
                    time.sleep(min(60.0, max(1.0, (end - t).total_seconds())))
                    continue
                if nxt is not None and (nxt - t).total_seconds() < est:
                    log(f"session (~{est:.0f} s) would run into the quiet window starting {nxt.isoformat(timespec='minutes')}: waiting")
                    time.sleep(min(60.0, max(1.0, (nxt - t).total_seconds() + 1)))
                    continue
                break
            profile = profiles[k % len(profiles)]
            k += 1
            rec = session_record(args, k, profile, log, windows)
            with open(os.path.join(RES, "sessions.jsonl"), "a") as f:
                f.write(json.dumps(rec) + "\n")
                f.flush()
                os.fsync(f.fileno())
            if rec.get("aborted"):
                break
    except Shutdown as e:
        log(f"run.py stopping: {e}")
    finally:
        kill_leftovers(log)
        write_json_atomic(os.path.join(RES, "progress.json"),
                          {"phase": "idle", "sessions_done": k, "updated": dt.datetime.now().astimezone().isoformat(timespec="seconds")})
        log("run.py exit")


def main():
    if len(sys.argv) == 3 and sys.argv[1] == "--worker":
        return worker_main(sys.argv[2])
    if len(sys.argv) == 3 and sys.argv[1] == "--recorder":
        return recorder_main(sys.argv[2])
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration-sim", type=float, default=300.0, help="sim seconds of workload per session (default 300)")
    ap.add_argument("--rtf", type=float, default=1.0, help="Gazebo real_time_factor (default 1.0; anything else makes timing_valid false)")
    ap.add_argument("--profile", default="default", help="comma list cycled per session: " + ", ".join(s.PROFILES))
    ap.add_argument("--sessions", type=int, default=1, help="number of sessions, 0 = until stopped (default 1)")
    ap.add_argument("--gpu", action="store_true", help="render on the GPU (container must be `sim.sh up N --gpu`)")
    ap.add_argument("--health-sources", default="fake", choices=["fake", "watcher"])
    ap.add_argument("--max-age", type=float, default=0.15, help="roster max_age_sec for the health components (default 0.15)")
    ap.add_argument("--rate-hz", type=float, default=20.0, help="fake component beat rate (default 20)")
    ap.add_argument("--soft-stop", type=float, default=0.15, help="stand-in soft stop timeout s (default 0.15)")
    ap.add_argument("--quiet-window", action="append", default=[], metavar="START/END")
    ap.add_argument("--label", default="")
    args = ap.parse_args()
    parent_main(args)


if __name__ == "__main__":
    main()
