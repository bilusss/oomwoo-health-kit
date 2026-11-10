"""Unattended long-run benchmark (no Gazebo): false trips and kill/restart cycles.

Run inside the container (see `tools/dev.sh soak-start`):
  python3 bench/soak.py [--passes N] [--quiet-window START/END ...] [--smoke]

Cells = load {idle, cpu, mixed} x roster max_age {0.10, 0.15, 0.25} s x component rate {20, 50} Hz, with the stand-in
soft stop at 0.15 s. Per cell: a steady-state window (default 10 min) counting false trips, then N kill/restart cycles
(default 40) alternating the two critical components. One JSON line per finished cell is appended and flushed to
results/soak.jsonl, so a crash loses at most the cell in progress. Events go to results/soak.log.

Load modes (never more than 12 worker processes in total, memory <= 2 GiB, disk file <= 256 MiB):
  idle   no load
  cpu    12 busy loops
  mixed  10 busy loops + 1 memory-churn + 1 disk-churn worker (12 total; the 12-process cap leaves no room for more)
         Python workers rather than stress-ng, so the benchmark does not depend on an apt install in the container.

Quiet windows ("2026-11-12T08:00/2026-11-12T22:00", local time of this process; set TZ in the container): inside a
window all load and child ROS processes are stopped and the script sleeps until the window ends. This is checked before
each cell and between kill cycles. A cell that cannot finish before the next window starts is not started.
SIGTERM/SIGINT/SIGHUP: stop everything, exit cleanly. The cell in progress is not recorded.
"""

import argparse
import datetime as dt
import json
import os
import signal
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ohk_bench as b  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(os.path.dirname(HERE), "results")
SOFT_STOP_S = 0.15
LOADS = ["idle", "cpu", "mixed"]
MAX_AGES = [0.10, 0.15, 0.25]
RATES = [20, 50]
WINDOW_S = 600.0
KILLS = 40
MEM_BYTES = 2 * 1024**3
DISK_BYTES = 256 * 1024**2
SETUP_ALLOWANCE_S = 90.0  # start-up and arming, for the "can it finish" estimate
PER_KILL_ALLOWANCE_S = 8.0  # sleep 1 s + stop latency + restart + 3 heartbeats to re-arm, with slack

BUSY = "while True: pass"
MEM_WORKER = (
    "import sys\nn=int(sys.argv[1]); b=bytearray(n); p=4096\n"
    "while True:\n"
    "    for i in range(0,n,p): b[i]=(b[i]+1)&255\n"
)
DISK_WORKER = (
    "import os,sys\nd,n=sys.argv[1],int(sys.argv[2]); blk=os.urandom(1<<20)\n"
    "while True:\n"
    "    with open(os.path.join(d,'churn.bin'),'wb') as f:\n"
    "        for _ in range(n>>20): f.write(blk)\n"
    "        f.flush(); os.fsync(f.fileno())\n"
)


class Shutdown(BaseException):
    pass


_stopping = False


def _on_signal(signum, _frame):
    global _stopping
    if _stopping:  # already unwinding; let the cleanup finish
        return
    _stopping = True
    raise Shutdown(signal.Signals(signum).name)


class Logger:
    def __init__(self, path):
        self.path = path
        os.makedirs(os.path.dirname(path), exist_ok=True)

    def __call__(self, msg):
        line = f"{dt.datetime.now().astimezone().isoformat(timespec='seconds')} {msg}"
        print(line, flush=True)
        with open(self.path, "a") as f:
            f.write(line + "\n")


class Windows:
    def __init__(self, specs):
        self.w = []
        for s in specs:
            a, _, z = s.partition("/")
            if not z:
                raise SystemExit(f"bad --quiet-window {s!r}; expected START/END")
            start, end = (self._parse(a), self._parse(z))
            if end <= start:
                raise SystemExit(f"quiet window {s!r}: end must be after start")
            self.w.append((start, end))

    @staticmethod
    def _parse(x):
        d = dt.datetime.fromisoformat(x)
        return d.astimezone() if d.tzinfo is None else d  # naive = local time of this process

    def active_end(self, now):
        ends = [e for s, e in self.w if s <= now < e]
        return max(ends) if ends else None

    def next_start(self, now):
        starts = [s for s, _ in self.w if s > now]
        return min(starts) if starts else None


def now():
    return dt.datetime.now().astimezone()


class Load:
    def __init__(self, mode):
        self.mode = mode
        self.procs = []
        self.tmp = None

    def _spawn(self, argv):
        self.procs.append(subprocess.Popen(argv, start_new_session=True, stdout=subprocess.DEVNULL,
                                           stderr=subprocess.DEVNULL))

    def start(self):
        busy = {"idle": 0, "cpu": 12, "mixed": 10}[self.mode]
        for _ in range(busy):
            self._spawn([sys.executable, "-c", BUSY])
        if self.mode == "mixed":
            self.tmp = tempfile.mkdtemp(prefix="ohk_soak_")
            self._spawn([sys.executable, "-c", MEM_WORKER, str(MEM_BYTES)])
            self._spawn([sys.executable, "-c", DISK_WORKER, self.tmp, str(DISK_BYTES)])
        assert len(self.procs) <= b.LOAD_PROCS

    def stop(self):
        for p in self.procs:
            try:
                os.killpg(p.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        for p in self.procs:
            try:
                p.wait(5)
            except subprocess.TimeoutExpired:
                pass
        self.procs = []
        if self.tmp:
            try:
                os.unlink(os.path.join(self.tmp, "churn.bin"))
            except FileNotFoundError:
                pass
            try:
                os.rmdir(self.tmp)
            except OSError:
                pass
            self.tmp = None


class Stack:
    """Load + aggregator + stand-in + roster + components + probe. Everything it starts, stop() ends."""

    def __init__(self, load, max_age, rate):
        self.load, self.max_age, self.rate = load, max_age, rate
        self.procs = self.probe = self.ex = None
        self.up = False

    def start(self, arm_timeout=40.0):
        import rclpy

        domain = 40 + (os.getpid() % 150)
        os.environ["ROS_DOMAIN_ID"] = str(domain)
        os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
        self.up = True
        self.load.start()
        time.sleep(0.5)
        self.procs = b.Procs(domain)
        b.kit_nodes(self.procs, self.max_age, self.rate,
                    mcu_extra=("-p", f"soft_stop_timeout_s:={SOFT_STOP_S}"))
        rclpy.init()
        self.probe = b.make_probe()
        self.ex, _ = b.spin_in_thread(self.probe)
        return b.wait_run(self.probe, arm_timeout)

    def stop(self):
        import rclpy

        if not self.up:
            return
        try:
            if self.ex:
                self.ex.shutdown()
            if self.probe:
                self.probe.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        if self.procs:
            self.procs.stop_all()
        self.load.stop()
        self.procs = self.probe = self.ex = None
        self.up = False

    def counters(self):
        with self.probe.lock:
            s = self.probe.states[-1][1] if self.probe.states else {}
        return s.get("soft_stops", 0), s.get("resets", 0)


def steady_window(stack, window):
    p = stack.probe
    ss0, rs0 = stack.counters()
    t0 = time.monotonic()
    time.sleep(window)
    t1 = time.monotonic()
    ss1, rs1 = stack.counters()
    _, _, _, hbs = p.snapshot()
    hb = [t for t in hbs if t >= t0]
    gaps = sorted((y - x) * 1000 for x, y in zip(hb, hb[1:]))
    return {
        "elapsed_s": round(t1 - t0, 2),
        "false_trips": (ss1 - ss0) + (rs1 - rs0),
        "soft_stops": ss1 - ss0,
        "cpu_resets": rs1 - rs0,
        "heartbeats": len(hb),
        "heartbeat_rate_hz": round(len(hb) / (t1 - t0), 2),
        "heartbeat_max_gap_ms": round(gaps[-1], 1) if gaps else None,
        "heartbeat_p99_gap_ms": round(gaps[int(0.99 * (len(gaps) - 1))], 1) if gaps else None,
    }


class KillLoop:
    """Bookkeeping that survives pauses (which restart the whole stack and its counters)."""

    def __init__(self):
        self.kills = []  # wall time of each SIGKILL
        self.latencies = []
        self.notes = []
        self.phases = [(time.time(), "steady")]  # (wall, label)
        self.soft_stop_ts = []  # wall time of every soft_stop transition message seen
        self.counter_soft = 0  # soft stops by the stand-in's cumulative counter, summed over stack lifetimes
        self.counter_reset = 0
        self.pauses = 0
        self.restarts = 0
        self.base = (0, 0)
        self.start_wall = time.time()

    def phase(self, label):
        self.phases.append((time.time(), label))

    def phase_at(self, ts):
        label = self.phases[0][1]
        for t, l in self.phases:
            if t <= ts:
                label = l
        return label

    def harvest(self, stack):
        """Fold the live stack's counters and soft-stop messages in, before it is stopped."""
        if not stack.probe:
            return
        ss, rs = stack.counters()
        self.counter_soft += ss - self.base[0]
        self.counter_reset += rs - self.base[1]
        off = time.time() - time.monotonic()
        with stack.probe.lock:
            self.soft_stop_ts += [(t + off, s.get("reason", "")) for t, s in stack.probe.states
                                  if s.get("state") == "soft_stop" and t + off >= self.start_wall]
        self.base = (0, 0)  # a fresh stand-in starts its counters at 0

    def rebase(self, stack):
        self.base = stack.counters()

    def summary(self):
        explained, unexplained = set(), []
        for ts, reason in sorted(set(self.soft_stop_ts)):
            hit = next((i for i, k in enumerate(self.kills) if i not in explained and k <= ts <= k + 3.0), None)
            if hit is None:
                unexplained.append({"time": dt.datetime.fromtimestamp(ts).astimezone().isoformat(timespec="milliseconds"),
                                    "phase": self.phase_at(ts), "reason": reason,
                                    "s_after_last_kill": round(ts - max([k for k in self.kills if k <= ts], default=ts), 3),
                                    "s_after_last_rearm": round(ts - max([t for t, l in self.phases
                                                                          if l == "steady" and t <= ts], default=ts), 3)})
            else:
                explained.add(hit)
        good = [x for x in self.latencies if x is not None]
        return {
            "kills": len(self.kills),
            "kill_times": [dt.datetime.fromtimestamp(k).astimezone().isoformat(timespec="milliseconds")
                           for k in self.kills],
            "kill_latencies_ms": self.latencies,
            "kill_latency_ms": b.summarize(good),
            "soft_stops_total": self.counter_soft,
            "cpu_resets_total": self.counter_reset,
            "unexplained_soft_stops": self.counter_soft - len(self.kills),
            "unexplained_listed": unexplained,
            "pauses": self.pauses,
            "stack_restarts_after_failure": self.restarts,
            "notes": self.notes,
        }


def pause_if_quiet(stack, kl, windows, log):
    """Between kill cycles: stop everything for the length of an active window, then bring the stack back."""
    end = windows.active_end(now())
    if end is None:
        return
    log(f"PAUSE (quiet window until {end.isoformat(timespec='minutes')}): stopping load and ROS processes")
    kl.harvest(stack)
    stack.stop()
    sleep_until(end, windows, log)
    log("RESUME: starting stack again")
    kl.pauses += 1
    kl.phase("restart")
    if not stack.start():
        kl.notes.append("stack did not arm after pause")
        raise RuntimeError("stack did not re-arm after pause")
    kl.rebase(stack)
    kl.phase("steady")


def sleep_until(end, windows, log):
    while True:
        left = (end - now()).total_seconds()
        if left <= 0:
            nxt = windows.active_end(now())
            if nxt is None:
                return
            end = nxt
            continue
        time.sleep(min(left, 30.0))


def kill_loop(stack, rate, n, windows, log):
    kl = KillLoop()
    kl.rebase(stack)
    victims = ["critical_a", "critical_b"]
    i = 0
    while i < n:
        pause_if_quiet(stack, kl, windows, log)
        p = stack.probe
        time.sleep(1.0)
        b.wait_for(lambda: p.state() == "run" and p.last_cmd()[1] and time.monotonic() - p.last_cmd()[0] < 0.1, 10.0)
        v = victims[i % 2]
        t0 = time.monotonic()
        kl.kills.append(time.time())
        kl.phase("restart")
        stack.procs.kill(v, signal.SIGKILL)
        if b.wait_for(lambda: any(not nz for t, nz in p.snapshot()[0] if t >= t0), 3.0):
            first = next(t for t, nz in p.snapshot()[0] if t >= t0 and not nz)
            kl.latencies.append(round((first - t0) * 1000, 1))
        else:
            kl.latencies.append(None)
            kl.notes.append(f"kill {i}: no zero cmd within 3 s")
        b.wait_for(lambda: p.state() != "run", 1.0)
        b.start_component(stack.procs, v, rate)
        if b.wait_run(p, 30.0):
            kl.phase("steady")
        else:
            kl.notes.append(f"kill {i}: did not re-arm within 30 s; restarting the whole stack")
            kl.harvest(stack)
            stack.stop()
            kl.restarts += 1
            if not stack.start():
                kl.notes.append("stack did not arm after restart; ending kill loop")
                break
            kl.rebase(stack)
            kl.phase("steady")
        i += 1
    kl.harvest(stack)
    return kl.summary()


def run_cell(cfg, window, kills, windows, log):
    load, max_age, rate = cfg
    rec = {"load": load, "max_age_sec": max_age, "rate_hz": rate, "soft_stop_s": SOFT_STOP_S,
           "window_s": window, "kills_requested": kills, "load_procs": {"idle": 0, "cpu": 12, "mixed": 12}[load],
           "started": now().isoformat(timespec="seconds")}
    stack = Stack(Load(load), max_age, rate)
    t0 = time.monotonic()
    try:
        if not stack.start():
            rec["armed"] = False
            return rec
        rec["armed"] = True
        rec["steady"] = steady_window(stack, window)
        log(f"  steady: {json.dumps(rec['steady'])}")
        rec["kill_loop"] = kill_loop(stack, rate, kills, windows, log)
    finally:
        stack.stop()
        rec["finished"] = now().isoformat(timespec="seconds")
        rec["duration_s"] = round(time.monotonic() - t0, 1)
    return rec


def wait_for_slot(est_s, windows, log):
    """Before a cell: sit out an active window; do not start a cell that cannot finish before the next one."""
    while True:
        t = now()
        end = windows.active_end(t)
        if end is not None:
            log(f"PAUSE before cell (quiet window until {end.isoformat(timespec='minutes')})")
            sleep_until(end, windows, log)
            log("RESUME")
            continue
        nxt = windows.next_start(t)
        if nxt is not None and t + dt.timedelta(seconds=est_s) > nxt:
            log(f"PAUSE before cell: ~{est_s / 60:.0f} min would not finish before the window at "
                f"{nxt.isoformat(timespec='minutes')}")
            sleep_until(nxt, windows, log)
            continue
        return


def agg_ref():
    try:
        return subprocess.check_output(["git", "-C", "/ws/src/health-monitor", "rev-parse", "HEAD"], text=True,
                                       stderr=subprocess.DEVNULL).strip()
    except Exception:
        return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--passes", type=int, default=0, help="number of passes over all cells (default 0 = forever)")
    ap.add_argument("--quiet-window", action="append", default=[], metavar="START/END",
                    help="local-time window, e.g. 2026-11-12T08:00/2026-11-12T22:00; repeatable")
    ap.add_argument("--smoke", action="store_true",
                    help="30 s windows, 3 kills, 3 cells (one per load mode), one pass; writes soak_smoke.* instead")
    ap.add_argument("--window-s", type=float, default=None, help=f"steady window per cell (default {WINDOW_S:.0f})")
    ap.add_argument("--kills", type=int, default=None, help=f"kill cycles per cell (default {KILLS})")
    a = ap.parse_args()

    window = a.window_s if a.window_s is not None else (30.0 if a.smoke else WINDOW_S)
    kills = a.kills if a.kills is not None else (3 if a.smoke else KILLS)
    passes = 1 if a.smoke else a.passes
    stem = "soak_smoke" if a.smoke else "soak"
    out_path = os.path.join(RES, f"{stem}.jsonl")
    log = Logger(os.path.join(RES, f"{stem}.log"))
    windows = Windows(a.quiet_window)

    cells = [(l, m, r) for l in LOADS for m in MAX_AGES for r in RATES]
    if a.smoke:
        cells = [("idle", 0.15, 20), ("cpu", 0.10, 50), ("mixed", 0.25, 20)]

    for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
        signal.signal(sig, _on_signal)

    log(f"start pid={os.getpid()} cells={len(cells)} passes={passes or 'forever'} window_s={window} kills={kills} "
        f"quiet_windows={[(s.isoformat(timespec='minutes'), e.isoformat(timespec='minutes')) for s, e in windows.w]} "
        f"aggregator={agg_ref()}")
    longest = 0.0
    p = 0
    try:
        while passes == 0 or p < passes:
            p += 1
            for cfg in cells:
                est = max(window + kills * PER_KILL_ALLOWANCE_S + SETUP_ALLOWANCE_S, 1.15 * longest)
                wait_for_slot(est, windows, log)
                log(f"pass {p} cell load={cfg[0]} max_age={cfg[1]} rate={cfg[2]}")
                rec = run_cell(cfg, window, kills, windows, log)
                rec["pass"] = p
                rec["aggregator"] = agg_ref()
                longest = max(longest, rec["duration_s"])
                with open(out_path, "a") as f:
                    f.write(json.dumps(rec) + "\n")
                    f.flush()
                    os.fsync(f.fileno())
                kl = rec.get("kill_loop", {})
                log(f"  done in {rec['duration_s']:.0f} s: false_trips={rec.get('steady', {}).get('false_trips')} "
                    f"unexplained_soft_stops={kl.get('unexplained_soft_stops')} notes={kl.get('notes')}")
        log("finished all passes")
    except Shutdown as e:
        log(f"stopping on {e}; the cell in progress is not recorded")
        return 143 if str(e) == "SIGTERM" else 130
    except Exception as e:  # keep the log useful if something unexpected kills an unattended run
        log(f"ERROR {type(e).__name__}: {e}")
        raise
    return 0


if __name__ == "__main__":
    sys.exit(main())
