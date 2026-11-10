"""Unattended long-run benchmark (no Gazebo): false trips and kill/restart cycles.

Run inside the container (see `tools/dev.sh soak-start` / `soak-stop`):
  python3 bench/soak.py [--passes N] [--quiet-window START/END ...] [--smoke]

Cells = load {idle, cpu, mixed} x roster max_age {0.10, 0.15, 0.25} s x component rate {20, 50} Hz, with the stand-in
soft stop at 0.15 s; the cell order is shuffled per pass (seed in the log). Per cell: stack start (stale Fast DDS shared
memory removed first), 30 s warm-up, a steady-state window (default 10 min) counting false trips, then N kill/restart
cycles (default 40) alternating the two critical components. One JSON line per finished cell is appended, flushed and
fsynced to results/soak.jsonl, so a crash loses at most the cell in progress. Events go to results/soak.log.

Validity: a steady window is valid only if the stand-in is in RUN at its end and the aggregate-heartbeat count and the
probe's cmd_vel count are each >= 95% of nominal (20 Hz); otherwise it carries "valid": false and no trip count. A cell
is invalid (valid=false, reason recorded) if /dev/shm free drops below 64 MiB or the stack cannot be armed.

Load modes (never more than 12 worker processes; the soak itself is not counted):
  idle   none
  cpu    12 busy loops
  mixed  10 busy loops + 1 worker rewriting a 1 GiB buffer (memory bandwidth, NOT memory pressure) + 1 worker writing
         about 20 MB/s to a 256 MiB file in the container's /tmp (no fsync)

Quiet windows ("2026-11-12T08:00/2026-11-12T22:00", local time of this process; set TZ): inside a window all load and
child ROS processes are stopped and the script sleeps until it ends. Checked before each cell and between kill cycles.
A cell that cannot finish before the next window starts is not started. Pause time is excluded from duration_s.
SIGTERM/SIGINT/SIGHUP: stop everything, exit. Children also die (PR_SET_PDEATHSIG) if this process is killed.
"""

import argparse
import datetime as dt
import fcntl
import glob
import json
import os
import random
import shutil
import signal
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ohk_bench as b  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
KIT_ROOT = os.path.dirname(HERE)
RES = os.path.join(KIT_ROOT, "results")
SOFT_STOP_S = 0.15
LOADS = ["idle", "cpu", "mixed"]
MAX_AGES = [0.10, 0.15, 0.25]
RATES = [20, 50]
WINDOW_S = 600.0
WARMUP_S = 30.0
KILLS = 40
NOMINAL_HZ = 20.0  # aggregate heartbeat (aggregator period fixed at 50 ms) and probe cmd_vel (50 ms timer)
MIN_FRACTION = 0.95
SHM_MIN_FREE = 64 * 1024 * 1024
REARM_TAIL_S = 5.0
SETUP_ALLOWANCE_S = 90.0
PER_KILL_ALLOWANCE_S = 8.0
LOCK_PATH = "/tmp/ohk_soak.lock"

LOAD_DESCRIPTION = {
    "idle": "no load",
    "cpu": "12 busy loops",
    "mixed": "10 busy loops + 1 worker rewriting a 1 GiB buffer (memory bandwidth, not memory pressure) + "
             "1 worker writing ~20 MB/s to a 256 MiB file in /tmp (no fsync); 12 worker processes",
}
LOAD_PROCS = {"idle": 0, "cpu": 12, "mixed": 12}

BUSY = "while True: pass"
MEM_WORKER = (
    "import sys\n"
    "n=int(sys.argv[1]); buf=bytearray(n); mv=memoryview(buf); blk=bytes(1<<20)\n"
    "while True:\n"
    "    for i in range(0,n,1<<20): mv[i:i+(1<<20)]=blk\n"
)
DISK_WORKER = (
    "import os,sys,time\n"
    "d=sys.argv[1]; limit=256<<20; blk=os.urandom(1<<20); rate=20\n"
    "f=open(os.path.join(d,'churn.bin'),'wb'); w=0; t=time.monotonic()\n"
    "while True:\n"
    "    f.write(blk); w+=1<<20\n"
    "    if w>=limit: f.flush(); f.seek(0); w=0\n"
    "    t+=1.0/rate; dly=t-time.monotonic()\n"
    "    if dly>0: time.sleep(dly)\n"
    "    else: t=time.monotonic()\n"
)


class Shutdown(BaseException):
    pass


class CellInvalid(Exception):
    pass


_stopping = False


def _on_signal(signum, _frame):
    global _stopping
    if _stopping:  # already unwinding; let the cleanup finish
        return
    _stopping = True
    raise Shutdown(signal.Signals(signum).name)


class Ctx:
    paused_s = 0.0


def now():
    return dt.datetime.now().astimezone()


def iso_mono(t, off):
    return dt.datetime.fromtimestamp(t + off).astimezone().isoformat(timespec="milliseconds")


class Logger:
    def __init__(self, path):
        self.path = path
        os.makedirs(os.path.dirname(path), exist_ok=True)

    def __call__(self, msg):
        line = f"{now().isoformat(timespec='seconds')} {msg}"
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
            start, end = self._parse(a), self._parse(z)
            if end <= start:
                raise SystemExit(f"quiet window {s!r}: end must be after start")
            self.w.append((start, end))

    @staticmethod
    def _parse(x):
        d = dt.datetime.fromisoformat(x)
        return d.astimezone() if d.tzinfo is None else d  # naive = local time of this process

    def active_end(self, t):
        ends = [e for s, e in self.w if s <= t < e]
        return max(ends) if ends else None

    def next_start(self, t):
        starts = [s for s, _ in self.w if s > t]
        return min(starts) if starts else None


# ---------------------------------------------------------------- shared memory

def shm_free():
    return shutil.disk_usage("/dev/shm").free


def _shm_in_use():
    """Paths under /dev/shm mapped or held open by any live process."""
    used = set()
    for pid in filter(str.isdigit, os.listdir("/proc")):
        if int(pid) == os.getpid():
            continue
        try:
            with open(f"/proc/{pid}/maps") as f:
                for line in f:
                    if "/dev/shm/" in line:
                        used.add(line.split()[-1])
            for fd in os.listdir(f"/proc/{pid}/fd"):
                try:
                    tgt = os.readlink(f"/proc/{pid}/fd/{fd}")
                except OSError:
                    continue
                if tgt.startswith("/dev/shm/"):
                    used.add(tgt)
        except OSError:
            continue
    return used


def clean_shm(log=None):
    """Remove Fast DDS segments/ports/semaphores that no live process maps. Call only with no stack running.

    `fastdds shm clean` does not help here: segments leaked by SIGKILLed participants look "in use" to it.
    """
    used = _shm_in_use()
    removed = freed = 0
    for path in glob.glob("/dev/shm/fastrtps_*") + glob.glob("/dev/shm/sem.fastrtps_*"):
        if path in used:
            continue
        try:
            size = os.stat(path).st_size
            os.unlink(path)
            removed += 1
            freed += size
        except OSError:
            pass
    if log and removed:
        log(f"  cleaned {removed} stale Fast DDS shm files ({freed / 1e6:.1f} MB); free now {shm_free() / 1e6:.0f} MB")
    return removed


def check_shm():
    free = shm_free()
    if free < SHM_MIN_FREE:
        raise CellInvalid(f"/dev/shm free {free / 1e6:.1f} MB < 64 MiB")


# ---------------------------------------------------------------- load and stack

class Load:
    def __init__(self, mode):
        self.mode = mode
        self.procs = []
        self.tmp = None

    def _spawn(self, argv):
        self.procs.append(subprocess.Popen(argv, start_new_session=True, stdout=subprocess.DEVNULL,
                                           stderr=subprocess.DEVNULL, preexec_fn=b.die_with_parent))

    def start(self):
        busy = {"idle": 0, "cpu": 12, "mixed": 10}[self.mode]
        for _ in range(busy):
            self._spawn([sys.executable, "-c", BUSY])
        if self.mode == "mixed":
            self.tmp = tempfile.mkdtemp(prefix="ohk_soak_", dir="/tmp")
            self._spawn([sys.executable, "-c", MEM_WORKER, str(1 << 30)])
            self._spawn([sys.executable, "-c", DISK_WORKER, self.tmp])
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
            shutil.rmtree(self.tmp, ignore_errors=True)
            self.tmp = None


class Stack:
    """Load + aggregator + stand-in + roster + components + probe. Everything it starts, stop() ends."""

    def __init__(self, load, max_age, rate, domain):
        self.load, self.max_age, self.rate, self.domain = load, max_age, rate, domain
        self.procs = self.probe = self.ex = None
        self.up = False

    def start(self, arm_timeout=40.0):
        import rclpy
        from std_msgs.msg import String

        from oomwoo_health_kit.qos import live_heartbeat_qos

        clean_shm()
        os.environ["ROS_DOMAIN_ID"] = str(self.domain)
        os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
        self.up = True
        self.load.start()
        time.sleep(0.5)
        self.procs = b.Procs(self.domain)
        b.kit_nodes(self.procs, self.max_age, self.rate, mcu_extra=("-p", f"soft_stop_timeout_s:={SOFT_STOP_S}"))
        rclpy.init()
        self.probe = b.make_probe()
        self.probe.comp_hbs = []  # (monotonic receipt time, component_id), for restart diagnostics

        def on_comp(m):
            try:
                cid = json.loads(m.data).get("component_id")
            except ValueError:
                return
            with self.probe.lock:
                self.probe.comp_hbs.append((time.monotonic(), cid))

        self.probe.create_subscription(String, "oomwoo/health/component", on_comp, live_heartbeat_qos())
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

    def dead_children(self):
        return [n for n, p in self.procs.by_name.items() if p.poll() is not None]


def sleep_checked(seconds):
    """Sleep in chunks, aborting the cell if /dev/shm runs low."""
    end = time.monotonic() + seconds
    while True:
        left = end - time.monotonic()
        if left <= 0:
            return
        time.sleep(min(left, 5.0))
        check_shm()


# ---------------------------------------------------------------- steady window

def steady_window(stack, window):
    p = stack.probe
    ss0, rs0 = stack.counters()
    t0 = time.monotonic()
    sleep_checked(window)
    t1 = time.monotonic()
    ss1, rs1 = stack.counters()
    cmds, _, _, hbs = p.snapshot()
    hb = [t for t in hbs if t0 <= t <= t1]
    cm = [t for t, _ in cmds if t0 <= t <= t1]
    gaps = sorted((y - x) * 1000 for x, y in zip(hb, hb[1:]))
    elapsed = t1 - t0
    expected = NOMINAL_HZ * elapsed
    out = {
        "elapsed_s": round(elapsed, 2),
        "heartbeats": len(hb), "heartbeat_rate_hz": round(len(hb) / elapsed, 2),
        "cmd_vel_msgs": len(cm), "cmd_vel_rate_hz": round(len(cm) / elapsed, 2),
        "nominal_hz": NOMINAL_HZ,
        "heartbeat_max_gap_ms": round(gaps[-1], 1) if gaps else None,
        "heartbeat_p99_gap_ms": round(gaps[int(0.99 * (len(gaps) - 1))], 1) if gaps else None,
        "end_state": p.state(),
    }
    reasons = []
    if out["end_state"] != "run":
        reasons.append(f"stand-in state {out['end_state']} at end of window")
    if len(hb) < MIN_FRACTION * expected:
        reasons.append(f"heartbeats {len(hb)} < 95% of nominal {expected:.0f}")
    if len(cm) < MIN_FRACTION * expected:
        reasons.append(f"cmd_vel msgs {len(cm)} < 95% of nominal {expected:.0f}")
    dead = stack.dead_children()
    if dead:
        reasons.append(f"child process exited: {dead}")
    if reasons:
        out["valid"] = False
        out["invalid_reason"] = "; ".join(reasons)
    else:
        out.update({"valid": True, "false_trips": (ss1 - ss0) + (rs1 - rs0), "soft_stops": ss1 - ss0,
                    "cpu_resets": rs1 - rs0})
    return out


# ---------------------------------------------------------------- kill loop

class KillLoop:
    """Bookkeeping across the whole loop, including pauses. All times are time.monotonic()."""

    def __init__(self, stack):
        self.off = time.time() - time.monotonic()
        self.t_start = time.monotonic()
        self.kills = []
        self.victims = []
        self.latencies = []
        self.skipped = []
        self.rearms = []
        self.restarts = []  # {cid, t, first_hb}
        self.softs = []  # {recv, t, source, reason, stack_ctx}
        self.cnt_soft = self.cnt_reset = 0
        self.base = stack.counters()  # first lifetime: count from the start of the loop; later ones start at 0
        self.pauses = 0
        self.failures = 0
        self.notes = []

    def harvest(self, stack):
        """Fold the live stack's counters and observations in, just before it is stopped."""
        if not stack.probe:
            return
        ss, rs = stack.counters()
        self.cnt_soft += ss - self.base[0]
        self.cnt_reset += rs - self.base[1]
        self.base = (0, 0)
        p = stack.probe
        with p.lock:
            states, stacks, comps = list(p.states), list(p.stacks), list(p.comp_hbs)
        for t, s in states:
            if s.get("state") != "soft_stop" or t < self.t_start:
                continue
            stamp = s.get("stamp")  # the stand-in's own steady-clock stamp at publication of the transition
            if isinstance(stamp, (int, float)) and abs(stamp - t) < 5.0:
                te, src = stamp, "standin_stamp"
            else:
                te, src = t, "probe_receipt"
            ctx = [{"age_s": round(te - ts, 3), **st} for ts, st in stacks if ts <= te + 0.05][-4:]
            self.softs.append({"recv": t, "t": te, "source": src, "reason": s.get("reason", ""), "stack_ctx": ctx})
        for r in self.restarts:
            if r["first_hb"] is None:
                r["first_hb"] = next((t for t, c in comps if c == r["cid"] and t >= r["t"]), None)

    def summary(self):
        claimed, unexplained = set(), []
        for s in sorted(self.softs, key=lambda x: x["t"]):
            hit = next((i for i, k in enumerate(self.kills) if i not in claimed and k <= s["t"] <= k + 3.0), None)
            if hit is not None:
                claimed.add(hit)
                continue
            t = s["t"]
            last_kill = max((k for k in self.kills if k <= t), default=None)
            last_rearm = max((r for r in self.rearms if r <= t), default=None)
            if last_kill is not None and (last_rearm is None or last_rearm < last_kill):
                phase = "restart"
            elif last_rearm is not None and t - last_rearm <= REARM_TAIL_S:
                phase = "rearm_tail"
            else:
                phase = "steady"
            lr = max((r for r in self.restarts if r["t"] <= t), key=lambda r: r["t"], default=None)
            ctx = s["stack_ctx"]
            at = ctx[-1] if ctx else {}
            blamed = sorted(set(at.get("stale_critical", []) + at.get("missing_critical", [])
                                + at.get("unhealthy_critical", [])))
            unexplained.append({
                "time": iso_mono(t, self.off), "time_source": s["source"], "phase": phase,
                "s_since_last_rearm": round(t - last_rearm, 3) if last_rearm is not None else None,
                "reason": s["reason"],
                "last_restart": None if lr is None else {
                    "component": lr["cid"], "s_since_restart_spawn": round(t - lr["t"], 3),
                    "s_since_its_first_heartbeat": None if lr["first_hb"] is None else round(t - lr["first_hb"], 3)},
                "aggregator_blamed_critical": blamed,
                "aggregator_stack_at_event": at, "aggregator_stack_recent": ctx,
            })
        good = [x for x in self.latencies if x is not None]
        out = {
            "kills": len(self.kills),
            "kills_skipped_not_steady": len(self.skipped),
            "kill_times": [iso_mono(k, self.off) for k in self.kills],
            "kill_victims": self.victims,
            "kill_latencies_ms": self.latencies,
            "kill_latency_ms": b.summarize(good),
            "soft_stops_total": self.cnt_soft,
            "cpu_resets_total": self.cnt_reset,
            "unexplained_soft_stops": self.cnt_soft - len(self.kills),
            "unexplained_listed": unexplained,
            "soft_stop_messages_seen": len(self.softs),
            "pauses": self.pauses,
            "stack_restarts_after_failure": self.failures,
            "notes": self.notes,
        }
        if self.skipped:
            out["unexplained_is_lower_bound"] = True
            out["skipped_not_steady"] = [iso_mono(t, self.off) for t in self.skipped]
        return out


def is_steady(p):
    c = p.last_cmd()
    return p.state() == "run" and c is not None and c[1] and time.monotonic() - c[0] < 0.1


def pause_if_quiet(stack, kl, windows, log):
    """Between kill cycles: stop everything for the length of an active window, then bring the stack back."""
    end = windows.active_end(now())
    if end is None:
        return
    t0 = time.monotonic()
    log(f"PAUSE (quiet window until {end.isoformat(timespec='minutes')}): stopping load and ROS processes")
    kl.harvest(stack)
    stack.stop()
    clean_shm()
    sleep_until(end, windows, log)
    log("RESUME: starting stack again")
    kl.pauses += 1
    try:
        armed = stack.start()
    finally:
        Ctx.paused_s += time.monotonic() - t0
    if not armed:
        raise CellInvalid("stack did not arm after pause")
    kl.rearms.append(time.monotonic())


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
    """Returns (summary, invalid_reason or None)."""
    kl = KillLoop(stack)
    victims = ["critical_a", "critical_b"]
    attempts = consecutive_skips = 0
    invalid = None
    try:
        while len(kl.kills) < n and attempts < 3 * n:
            attempts += 1
            check_shm()
            pause_if_quiet(stack, kl, windows, log)
            p = stack.probe
            time.sleep(1.0)
            b.wait_for(lambda: is_steady(p), 10.0)
            if not is_steady(p):
                kl.skipped.append(time.monotonic())
                consecutive_skips += 1
                if consecutive_skips >= 5:
                    kl.notes.append("5 consecutive kills skipped (not steady); kill loop ended")
                    invalid = "kill loop ended: system never steady before a kill"
                    break
                continue
            consecutive_skips = 0
            v = victims[len(kl.kills) % 2]
            t0 = time.monotonic()
            kl.kills.append(t0)
            kl.victims.append(v)
            stack.procs.kill(v, signal.SIGKILL)
            if b.wait_for(lambda: any(not nz for t, nz in p.snapshot()[0] if t >= t0), 3.0):
                first = next(t for t, nz in p.snapshot()[0] if t >= t0 and not nz)
                kl.latencies.append(round((first - t0) * 1000, 1))
            else:
                kl.latencies.append(None)
                kl.notes.append(f"kill {len(kl.kills) - 1}: no zero cmd within 3 s")
            b.wait_for(lambda: p.state() != "run", 1.0)
            b.start_component(stack.procs, v, rate)
            kl.restarts.append({"cid": v, "t": time.monotonic(), "first_hb": None})
            if b.wait_run(p, 30.0):
                kl.rearms.append(time.monotonic())
            else:
                kl.notes.append(f"kill {len(kl.kills) - 1}: did not re-arm within 30 s; restarting the whole stack")
                kl.harvest(stack)
                stack.stop()
                clean_shm()
                kl.failures += 1
                if not stack.start():
                    raise CellInvalid("stack did not arm after a failed re-arm")
                kl.rearms.append(time.monotonic())
    except CellInvalid as e:
        invalid = str(e)
        kl.notes.append(f"aborted: {invalid}")
    try:
        kl.harvest(stack)
    except Exception as e:  # stack may be gone
        kl.notes.append(f"final harvest failed: {e}")
    return kl.summary(), invalid


# ---------------------------------------------------------------- cells

def git_rev(path):
    try:
        return subprocess.check_output(["git", "-c", "safe.directory=*", "-C", path, "rev-parse", "HEAD"], text=True,
                                       stderr=subprocess.DEVNULL).strip()
    except Exception:
        return None


def run_cell(cfg, args, windows, log, meta):
    load, max_age, rate = cfg
    Ctx.paused_s = 0.0
    rec = dict(meta, load=load, load_description=LOAD_DESCRIPTION[load], max_age_sec=max_age, rate_hz=rate,
               soft_stop_s=SOFT_STOP_S, warmup_s=args.warmup_s, window_s=args.window_s,
               kills_requested=args.kills, load_procs=LOAD_PROCS[load],
               loadavg_start=[round(x, 2) for x in os.getloadavg()], started=now().isoformat(timespec="seconds"),
               valid=True)
    stack = Stack(Load(load), max_age, rate, args.domain)
    t0 = time.monotonic()
    try:
        clean_shm(log)
        rec["shm_free_start_bytes"] = shm_free()
        if not stack.start():
            raise CellInvalid("stack did not arm within 40 s")
        sleep_checked(args.warmup_s)
        if not b.wait_run(stack.probe, 10.0):
            raise CellInvalid("stand-in not in RUN after warm-up")
        rec["steady"] = steady_window(stack, args.window_s)
        s = rec["steady"]
        log(f"  steady: valid={s['valid']} " + (f"false_trips={s['false_trips']}" if s["valid"] else s["invalid_reason"])
            + f" hb={s['heartbeats']} cmds={s['cmd_vel_msgs']} max_gap_ms={s['heartbeat_max_gap_ms']}")
        rec["kill_loop"], invalid = kill_loop(stack, rate, args.kills, windows, log)
        if invalid:
            raise CellInvalid(invalid)
    except CellInvalid as e:
        rec["valid"] = False
        rec["invalid_reason"] = str(e)
        log(f"  CELL INVALID: {e}")
    finally:
        try:
            rec["shm_free_end_bytes"] = shm_free()
        finally:
            stack.stop()
            clean_shm()
            rec["shm_free_after_cleanup_bytes"] = shm_free()
            rec["finished"] = now().isoformat(timespec="seconds")
            rec["paused_s"] = round(Ctx.paused_s, 1)
            rec["duration_s"] = round(time.monotonic() - t0 - Ctx.paused_s, 1)
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


def parse_cell(s):
    try:
        load, ma, rate = s.split(":")
        assert load in LOADS
        return (load, float(ma), int(rate))
    except Exception:
        raise SystemExit(f"bad --cell {s!r}; expected LOAD:MAX_AGE:RATE, e.g. cpu:0.10:50")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--passes", type=int, default=0, help="number of passes over all cells (default 0 = forever)")
    ap.add_argument("--quiet-window", action="append", default=[], metavar="START/END",
                    help="local-time window, e.g. 2026-11-12T08:00/2026-11-12T22:00; repeatable")
    ap.add_argument("--smoke", action="store_true",
                    help="30 s windows, 5 s warm-up, 3 kills, 3 cells (one per load mode), one pass; writes soak_smoke.*")
    ap.add_argument("--cell", action="append", default=[], metavar="LOAD:MAX_AGE:RATE",
                    help="run only these cells (repeatable), e.g. cpu:0.10:50")
    ap.add_argument("--window-s", type=float, default=None, help=f"steady window per cell (default {WINDOW_S:.0f})")
    ap.add_argument("--warmup-s", type=float, default=None, help=f"armed warm-up before the window (default {WARMUP_S:.0f})")
    ap.add_argument("--kills", type=int, default=None, help=f"kill cycles per cell (default {KILLS})")
    ap.add_argument("--seed", type=int, default=None, help="cell-order shuffle seed (default: time-based, logged)")
    args = ap.parse_args()

    args.window_s = args.window_s if args.window_s is not None else (30.0 if args.smoke else WINDOW_S)
    args.warmup_s = args.warmup_s if args.warmup_s is not None else (5.0 if args.smoke else WARMUP_S)
    args.kills = args.kills if args.kills is not None else (3 if args.smoke else KILLS)
    passes = 1 if args.smoke else args.passes
    stem = "soak_smoke" if args.smoke else "soak"
    out_path = os.path.join(RES, f"{stem}.jsonl")
    log = Logger(os.path.join(RES, f"{stem}.log"))
    windows = Windows(args.quiet_window)
    args.domain = 10 + os.getpid() % 90  # ROS_DOMAIN_ID must stay within 0-101

    lock = open(LOCK_PATH, "w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        raise SystemExit("another soak.py is already running (lock held on " + LOCK_PATH + ")")

    cells = [(l, m, r) for l in LOADS for m in MAX_AGES for r in RATES]
    if args.smoke:
        cells = [("idle", 0.15, 20), ("cpu", 0.10, 50), ("mixed", 0.25, 20)]
    if args.cell:
        cells = [parse_cell(c) for c in args.cell]

    for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
        signal.signal(sig, _on_signal)

    run_id = now().strftime("%Y%m%dT%H%M%S") + f"-{os.getpid()}"
    seed = args.seed if args.seed is not None else int(time.time())
    agg = os.environ.get("OHK_AGG_COMMIT") or git_rev("/ws/src/health-monitor")
    kit = os.environ.get("OHK_KIT_COMMIT") or git_rev(KIT_ROOT)
    log(f"start run_id={run_id} pid={os.getpid()} cells={len(cells)} passes={passes or 'forever'} "
        f"warmup_s={args.warmup_s} window_s={args.window_s} kills={args.kills} domain={args.domain} seed={seed} "
        f"quiet_windows={[(s.isoformat(timespec='minutes'), e.isoformat(timespec='minutes')) for s, e in windows.w]} "
        f"aggregator={agg} kit={kit} shm_free_mb={shm_free() / 1e6:.0f}")
    longest = 0.0
    p = 0
    try:
        while passes == 0 or p < passes:
            p += 1
            order = list(cells)
            random.Random(seed + p).shuffle(order)
            log(f"pass {p} order (seed {seed + p}): " + " ".join(f"{l}:{m}:{r}" for l, m, r in order))
            for idx, cfg in enumerate(order):
                est = max(args.warmup_s + args.window_s + args.kills * PER_KILL_ALLOWANCE_S + SETUP_ALLOWANCE_S,
                          1.15 * longest)
                wait_for_slot(est, windows, log)
                log(f"pass {p} cell {idx + 1}/{len(order)} load={cfg[0]} max_age={cfg[1]} rate={cfg[2]}")
                meta = {"run_id": run_id, "pass": p, "cell_index": idx, "cell_order_seed": seed + p,
                        "aggregator_commit": agg, "kit_commit": kit}
                rec = run_cell(cfg, args, windows, log, meta)
                longest = max(longest, rec["duration_s"])
                with open(out_path, "a") as f:
                    f.write(json.dumps(rec) + "\n")
                    f.flush()
                    os.fsync(f.fileno())
                kl = rec.get("kill_loop", {})
                log(f"  done in {rec['duration_s']:.0f} s (+{rec['paused_s']:.0f} s paused): valid={rec['valid']} "
                    f"unexplained_soft_stops={kl.get('unexplained_soft_stops')} skipped={kl.get('kills_skipped_not_steady')} "
                    f"shm_free_mb={rec['shm_free_end_bytes'] / 1e6:.0f}")
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
