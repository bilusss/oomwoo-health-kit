"""Review experiment (scratch): non-roster silent rclpy node started, kept 2.5 s, then stopped by SIGINT (clean, waits
for exit) or SIGKILL. Same stack and metrics as bench/diag/restart_diag.py extra_silent. One JSON line per cycle.

  python3 stopmode.py --stop int|kill --cycles 70 --domain 61 --out FILE [--trace DIR]
"""
import argparse, glob, json, os, signal, subprocess, sys, threading, time

ROOT = "/ws/src/oomwoo_health_kit"
sys.path.insert(0, os.path.join(ROOT, "bench"))
sys.path.insert(0, os.path.join(ROOT, "bench", "diag"))
import ohk_bench as b  # noqa: E402
import soak  # noqa: E402
from restart_diag import make_probe  # noqa: E402

# Clean shutdown on SIGINT: leave the loop, destroy the node, shut rclpy down, exit normally (status 0).
SILENT = ("import rclpy, sys\nfrom rclpy.node import Node\nrclpy.init()\nn = Node('diag_silent')\n"
          "try:\n    while rclpy.ok():\n        rclpy.spin_once(n, timeout_sec=0.1)\nexcept BaseException:\n    pass\n"
          "try:\n    n.destroy_node()\nexcept BaseException:\n    pass\nrclpy.try_shutdown()\nsys.exit(0)\n")


def shm_names():
    return sorted(os.path.basename(p) for p in glob.glob("/dev/shm/*fastrtps*"))


def run(a):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    soak.clean_shm()
    os.environ["ROS_DOMAIN_ID"] = str(a.domain)
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    procs = b.Procs(a.domain)
    if a.trace:
        os.makedirs(a.trace, exist_ok=True)
        procs.env["LD_PRELOAD"] = "/tmp/sleep_trace.so"
        procs.env["SLEEP_TRACE_DIR"] = a.trace
    out = open(a.out, "a")
    try:
        b.kit_nodes(procs, 0.15, 20, mcu_extra=("-p", "soft_stop_timeout_s:=0.15", "-p", "rearm_count:=3"))
        rclpy.init()
        p = make_probe()
        ex = SingleThreadedExecutor(); ex.add_node(p)
        threading.Thread(target=ex.spin, daemon=True).start()
        if not b.wait_for(p.steady, 40):
            raise SystemExit("stack did not arm")
        time.sleep(5.0)
        done = skipped = 0
        while done < a.cycles and skipped < 30:
            time.sleep(1.0)
            b.wait_for(p.steady, 10)
            if not p.steady():
                skipped += 1
                continue
            t0 = time.monotonic()
            shm0 = shm_names()
            pr = procs.start("extra", [sys.executable, "-c", SILENT])
            rec = {"i": done, "stop": a.stop, "t0": t0, "pids_before": {n: x.pid for n, x in procs.by_name.items()}}
            time.sleep(2.5)
            ts = time.monotonic()
            procs.kill("extra", signal.SIGINT if a.stop == "int" else signal.SIGKILL)
            try:
                rc = pr.wait(5)
            except subprocess.TimeoutExpired:
                procs.kill("extra", signal.SIGKILL); rc = pr.wait(2); rec["int_timeout"] = True
            te = time.monotonic()
            rec.update(t_stop=round(ts - t0, 4), t_exit=round(te - t0, 4), rc=rc)
            time.sleep(3.5)
            t1 = time.monotonic()
            shm1 = shm_names()
            rec["shm_new_after"] = [x for x in shm1 if x not in shm0]
            evs = p.take(t0 - 0.5)
            rec["t1"] = t1
            rec["ev"] = [(round(t - t0, 4), k, d) for t, k, d in evs if t <= t1]
            soft = [round(t - t0, 4) for t, k, d in evs if k == "state" and d["s"] == "soft_stop" and t >= t0]
            rec["flaps"] = soft
            stk = [t for t, k, d in evs if k == "stack" and t >= t0 - 0.5]
            rec["stack_gaps"] = [(round(x - t0, 3), round(y - x, 3)) for x, y in zip(stk, stk[1:]) if y - x > 0.2]
            mcu = [t for t, k, d in evs if k == "mcu" and t >= t0 - 0.5]
            rec["mcu_gaps"] = [(round(x - t0, 3), round(y - x, 3)) for x, y in zip(mcu, mcu[1:]) if y - x > 0.2]
            out.write(json.dumps(rec) + "\n"); out.flush()
            done += 1
            if done % 10 == 0:
                print(f"[{a.stop}] {done}/{a.cycles}", flush=True)
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass
        procs.stop_all()
        soak.clean_shm()
        out.close()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--stop", choices=["int", "kill"], required=True)
    ap.add_argument("--cycles", type=int, default=70)
    ap.add_argument("--domain", type=int, default=61)
    ap.add_argument("--out", required=True)
    ap.add_argument("--trace")
    run(ap.parse_args())
