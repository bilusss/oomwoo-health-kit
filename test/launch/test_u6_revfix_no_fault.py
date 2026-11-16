"""M8 rule B, no-fault cases: reversing with fixes every 0.6 s including a delayed fix (gap up to 1.15 s; the recon has 1.2 s gaps in 3.8%); the
reversing command ends with a zero; then driving forward with NO fix publisher at all (the rule applies to reversing only)."""

import os
import signal
import time

import ohk_nav_harness as n


def generate_test_description():
    return n.m8_description("revfix", fakes=("scan", "odom", "tf", "polygon", "fix"))


class TestRevfixNoFault(n.NavCase):
    PROBE_FEED = False

    def _reversing_or_driving_run(self):
        self.wait_for(lambda: self.current_state() == "run" and any(nz for _, nz in self.cmds_since(time.monotonic() - 0.3)), 10.0,
                      "RUN with a non-zero command")

    def test_reverse_with_a_missed_fix_and_forward_without_fixes_do_not_trip(self):
        self.wait_quiet_run()
        rev = self.spawn_fake("cmd", ("--value", "-0.06"))
        self.set_enable(True)
        self._reversing_or_driving_run()      # the start-up of a new publisher (first, late messages) is not part of the no-fault window
        time.sleep(1.5)
        t0 = time.monotonic()
        time.sleep(1.5)
        fix = self.pid("fix")
        os.kill(fix, signal.SIGSTOP)      # a delayed fix: the gap is at most one period (0.6 s) + 0.55 s = 1.15 s, well inside the bound
        time.sleep(0.55)
        os.kill(fix, signal.SIGCONT)
        time.sleep(4.0)
        t_end_rev = self.set_enable(False)  # dock_drive ends the reversing with a zero
        time.sleep(1.0)
        rev.kill()
        os.kill(fix, signal.SIGKILL)      # no fix publisher any more
        self.spawn_fake("cmd", ("--value", "0.2"))
        self.set_enable(True)             # forward driving
        self._reversing_or_driving_run()
        time.sleep(1.5)                   # a new publisher's first messages arrive in a burst after discovery (stale for drive_supervision): not part of the window
        t1 = time.monotonic()
        time.sleep(4.0)
        windows = [(t0, t_end_rev), (t1, time.monotonic())]
        faults = [s for w0, w1 in windows for ts, s in self.stacks_since(w0) if ts <= w1 and s["state"] == "fault"]
        beats = [ts for ts, c, _ in self.probe.comp_details if c == "dock_fix" and ts >= t0]
        gaps = [b - a for a, b in zip(beats, beats[1:])]
        self.record("u6_revfix_no_fault", beats=len(beats), max_beat_gap_s=max(gaps) if gaps else None, fault_stacks=len(faults))
        self.assertEqual([(s["stale_critical"], s["unhealthy_critical"], s["missing_critical"]) for s in faults], [], "no fault in the no-fault windows (reversing with a delayed fix; forward without fixes)")
        self.assertTrue(all(s["state"] == "run" for w0, w1 in windows for ts, s in self.states_since(w0) if ts <= w1), "the stand-in never left RUN")
