"""Part A: false trips and kill latency under CPU load, no Gazebo.

  python3 bench/part_a.py sweep            # all configs, writes results/part_a_trips.json
  python3 bench/part_a.py kills            # kill latency for configs with 0 false trips under load
  python3 bench/part_a.py one --load 12 --max-age 0.15 --rate 20 [--kills 5]   # single config, prints JSON

Run inside the container: tools/dev.sh exec 'python3 /ws/src/oomwoo_health_kit/bench/part_a.py sweep'
"""

import argparse
import json
import os
import signal
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ohk_bench as b  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(os.path.dirname(HERE), "results")
MAX_AGES = [0.10, 0.15, 0.25, 0.50]
RATES = [10, 20, 50]
LOADS = [0, 12]
WINDOW_S = 60.0


def run_one(load, max_age, rate, kills=0, window=WINDOW_S, domain=None):
    import rclpy

    domain = domain or (40 + (os.getpid() % 150))
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    out = {"load_procs": load, "max_age_sec": max_age, "rate_hz": rate, "window_s": window}
    procs = b.Procs(domain)
    probe = ex = None
    with b.CpuLoad(load):
        try:
            time.sleep(0.5)
            b.kit_nodes(procs, max_age, rate)
            rclpy.init()
            probe = b.make_probe()
            ex, _ = b.spin_in_thread(probe)
            if not b.wait_run(probe, 40.0):
                out["armed"] = False
                return out
            out["armed"] = True
            if kills:
                out.update(kill_runs(procs, probe, rate, kills))
            else:
                out.update(trip_window(probe, window))
        finally:
            if ex:
                ex.shutdown()
            if probe:
                probe.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            procs.stop_all()
    return out


def trip_window(probe, window):
    t0 = time.monotonic()
    time.sleep(window)
    t1 = time.monotonic()
    cmds, states, stacks, hbs = probe.snapshot()
    hb = [t for t in hbs if t >= t0]
    gaps = [(y - x) * 1000 for x, y in zip(hb, hb[1:])]
    s_in = [s for t, s in states if t >= t0]
    trips_msgs = sum(1 for s in s_in if s.get("from") == "run")
    last = states[-1][1]
    stack_in = [s for t, s in stacks if t >= t0]
    agg_faults = sum(1 for a, c in zip(stack_in, stack_in[1:]) if a["state"] != "fault" and c["state"] == "fault")
    zero_cmds = sum(1 for t, nz in cmds if t >= t0 and not nz)
    return {
        "elapsed_s": round(t1 - t0, 2),
        "false_trips": max(trips_msgs, last["soft_stops"], last["resets"]),
        "trips_seen_in_state_msgs": trips_msgs,
        "mcu_soft_stops_total": last["soft_stops"],
        "mcu_resets_total": last["resets"],
        "aggregator_fault_entries": agg_faults,
        "zero_cmd_msgs_in_window": zero_cmds,
        "heartbeats": len(hb),
        "heartbeat_rate_hz": round(len(hb) / (t1 - t0), 2),
        "heartbeat_max_gap_ms": round(max(gaps), 1) if gaps else None,
        "heartbeat_p99_gap_ms": round(sorted(gaps)[int(0.99 * (len(gaps) - 1))], 1) if gaps else None,
    }


def kill_runs(procs, probe, rate, n):
    lat, notes, ss_delta = [], [], []
    victims = ["critical_a", "critical_b"]
    with probe.lock:
        ss_start = probe.states[-1][1]["soft_stops"]
    for i in range(n):
        time.sleep(1.0)
        v = victims[i % 2]
        # the loop must be steady before the kill: RUN, non-zero output in the last 100 ms
        b.wait_for(lambda: probe.state() == "run" and probe.last_cmd()[1] and time.monotonic() - probe.last_cmd()[0] < 0.1,
                   10.0)
        with probe.lock:
            ss0 = probe.states[-1][1]["soft_stops"]
        t0 = time.monotonic()
        procs.kill(v, signal.SIGKILL)
        ok = b.wait_for(lambda: any(not nz for t, nz in probe.snapshot()[0] if t >= t0), 3.0)
        if not ok:
            notes.append(f"kill {i}: no zero cmd within 3 s")
            lat.append(None)
        else:
            first = next(t for t, nz in probe.snapshot()[0] if t >= t0 and not nz)
            lat.append(round((first - t0) * 1000, 1))
        b.wait_for(lambda: probe.state() != "run", 1.0)
        with probe.lock:
            ss_delta.append(probe.states[-1][1]["soft_stops"] - ss0)
        b.start_component(procs, v, rate)
        if not b.wait_run(probe, 30.0):
            notes.append(f"kill {i}: did not re-arm within 30 s")
            break
    good = [x for x in lat if x is not None]
    with probe.lock:
        ss_total = probe.states[-1][1]["soft_stops"] - ss_start
    # soft stops over the whole loop, including the restart phases, minus one expected per kill
    return {"soft_stops_added_per_kill": ss_delta, "soft_stops_total": ss_total,
            "unexplained_soft_stops": ss_total - len(lat),
            "kill_latencies_ms": lat, "kill_latency_ms": b.summarize(good), "notes": notes}


def skipped(max_age, rate):
    return 1.0 / rate >= max_age


def sweep(path):
    results = []
    if os.path.exists(path):
        results = json.load(open(path))["runs"]
    done = {(r["load_procs"], r["max_age_sec"], r["rate_hz"]) for r in results}
    skips = []
    for load in LOADS:
        for ma in MAX_AGES:
            for rate in RATES:
                if skipped(ma, rate):
                    skips.append({"load_procs": load, "max_age_sec": ma, "rate_hz": rate,
                                  "reason": f"period {1000 / rate:.0f} ms >= max_age {ma * 1000:.0f} ms"})
                    continue
                if (load, ma, rate) in done:
                    continue
                print(f"load={load} max_age={ma} rate={rate}", flush=True)
                r = run_one(load, ma, rate)
                print(json.dumps(r), flush=True)
                results.append(r)
                save(path, results, skips)
    save(path, results, skips)


def save(path, runs, skips):
    os.makedirs(RES, exist_ok=True)
    json.dump({"script": "bench/part_a.py sweep", "window_s": WINDOW_S, "runs": runs, "skipped": skips},
              open(path, "w"), indent=1)


def kills(trips_path, path):
    trips = json.load(open(trips_path))["runs"]
    runs = []
    for r in trips:
        if r["load_procs"] == 12 and r.get("armed") and r["false_trips"] == 0:
            print(f"kills max_age={r['max_age_sec']} rate={r['rate_hz']}", flush=True)
            k = run_one(12, r["max_age_sec"], r["rate_hz"], kills=5)
            print(json.dumps(k), flush=True)
            runs.append(k)
            json.dump({"script": "bench/part_a.py kills", "load_procs": 12, "runs": runs}, open(path, "w"), indent=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["sweep", "kills", "one"])
    ap.add_argument("--load", type=int, default=0)
    ap.add_argument("--max-age", type=float, default=0.15)
    ap.add_argument("--rate", type=int, default=20)
    ap.add_argument("--kills", type=int, default=0)
    ap.add_argument("--window", type=float, default=WINDOW_S)
    a = ap.parse_args()
    if a.cmd == "sweep":
        sweep(os.path.join(RES, "part_a_trips.json"))
    elif a.cmd == "kills":
        kills(os.path.join(RES, "part_a_trips.json"), os.path.join(RES, "part_a_kills.json"))
    else:
        print(json.dumps(run_one(a.load, a.max_age, a.rate, a.kills, a.window), indent=1))


if __name__ == "__main__":
    main()
