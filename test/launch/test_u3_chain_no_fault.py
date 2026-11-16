"""M8 rule A, no-fault cases: a live commander, and three commanded stops (the commander sends one zero and goes quiet, the smoother
decelerates over 0.3 s as in the M8 recon). The stand-in must stay in RUN, the command_chain component must keep beating, and the
aggregator must never withhold the heartbeat."""

import time

import ohk_nav_harness as n


def generate_test_description():
    return n.m8_description("chain", fakes=("scan", "odom", "tf", "polygon", "smoother", "commander"))


class TestChainNoFault(n.NavCase):
    PROBE_FEED = False

    def test_live_commander_and_commanded_stops_do_not_trip(self):
        self.wait_quiet_run()
        t0 = time.monotonic()
        for _ in range(3):
            self.set_enable(True)
            time.sleep(3.0)
            self.set_enable(False)      # one zero on the input; the output ramps down over 0.3 s, then silence
            time.sleep(2.0)
        with self.probe.lock:
            beats = [ts for ts, c, _ in self.probe.comp_details if c == "command_chain" and ts >= t0]
        bad = [s for _, s in self.stacks_since(t0) if s["state"] == "fault"]
        gaps = [b - a for a, b in zip(beats, beats[1:])]
        self.record("u3_chain_no_fault", beats=len(beats), max_beat_gap_s=max(gaps) if gaps else None, fault_stacks=len(bad),
                    non_run_states=sum(1 for _, s in self.states_since(t0) if s["state"] != "run"))
        self.assertGreater(len(beats), 100, "command_chain must keep beating (idle, driving and decelerating)")
        self.assertEqual(bad, [], "no fault stack message in the no-fault run")
        self.assertTrue(all(s["state"] == "run" for _, s in self.states_since(t0)), "the stand-in never left RUN")
