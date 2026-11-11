"""Frozen /clock seen by the AGGREGATOR (contract: a frozen ROS clock is a fault, no MCU heartbeat).

Every node runs with use_sim_time and this test publishes /clock at 100 Hz of sim time. After RUN the clock stops
while the components keep going. What the components do with a frozen clock: their work loop sleeps on wall time, so
they keep publishing at 20 Hz, but each beat is stamped with the node's ROS clock, so every stamp_sec repeats the
last /clock value (asserted below: no stamp advances after the freeze). The aggregator's own 50 ms timer runs on the
ROS clock too; with /clock stopped, rclpy does not fire it at all, so the question is whether it publishes anything
more. Expected by the contract: no further oomwoo/health/mcu_heartbeat, and the robot stops. The stand-in uses
clock_source=steady (its own detection of a frozen clock is off), so the stop must come from the withheld heartbeats.

Assertions: premise (components keep beating with frozen stamps), no mcu_heartbeat after the freeze (grace for
messages already in flight), no stack message that still says emit_mcu_heartbeat, and zero cmd_vel that stays zero.
The last latched stack state is recorded, not asserted.

Known result per aggregator ref goes in EXPECTED_FAIL (sha prefix -> reason); the test then must fail, and an
unexpected pass is reported as such by unittest.
"""

import time
import unittest

import ohk_harness as h
from rosgraph_msgs.msg import Clock

# aggregator commit (prefix of the sha in /ws/.agg_built) -> why the contract is not met
EXPECTED_FAIL = {}


def _agg_sha():
    try:
        return open("/ws/.agg_built").read().strip()
    except OSError:
        return ""


def expected_fail(fn):
    sha = _agg_sha()
    for prefix in EXPECTED_FAIL:
        if sha.startswith(prefix):
            return unittest.expectedFailure(fn)
    return fn


def generate_test_description():
    return h.build_description(sim_time=True, mcu_params={"clock_source": "steady"})


class TestFrozenClockAggregator(h.OhkCase):
    GRACE_S = 0.12  # messages already published or in flight when the clock stops

    @expected_fail
    def test_frozen_clock_withholds_mcu_heartbeat(self):
        clock_pub = self.probe.create_publisher(Clock, "/clock", 10)
        state = {"t": 1000.0, "run": True}

        def tick():
            if state["run"]:
                state["t"] += 0.01
                m = Clock()
                m.clock.sec = int(state["t"])
                m.clock.nanosec = int((state["t"] % 1) * 1e9)
                clock_pub.publish(m)

        timer = self.probe.create_timer(0.01, tick)
        try:
            self.wait_run(30.0)
            time.sleep(1.0)  # steady RUN with live heartbeats first
            self.assertTrue(self.mcu_hbs_since(time.monotonic() - 0.5), "no aggregate heartbeat before the freeze")

            state["run"] = False  # /clock stops; components keep beating, stamps stop advancing
            t0 = time.monotonic()
            self.wait_for(lambda: any(not nz for _, nz in self.cmds_since(t0)), 3.0, "zero cmd_vel")
            lat = next(t for t, nz in self.cmds_since(t0) if not nz) - t0
            time.sleep(1.5)
            tg = t0 + self.GRACE_S

            # premise: components still publish, with stamps frozen at the last /clock value
            with self.probe.lock:
                stamps = [(t, c, s) for t, c, s in self.probe.comp_stamps if t >= tg]
            for cid in ("critical_a", "critical_b"):
                mine = [s for _, c, s in stamps if c == cid]
                self.assertGreater(len(mine), 10, f"{cid} stopped beating")
                self.assertEqual(len(set(mine)), 1, f"{cid} stamp advanced although /clock stopped")

            hbs = self.mcu_hbs_since(tg)
            stacks = self.stacks_since(tg)
            last = self.last_stack()
            self.record(
                "k_frozen_clock_aggregator",
                latency_ms=lat * 1000,
                mcu_heartbeats_after_freeze=len(hbs),
                stack_msgs_after_freeze=len(stacks),
                last_stack_state=last and last.get("state"),
                last_stack_emit=last and last.get("emit_mcu_heartbeat"),
            )
            self.assertEqual(len(hbs), 0, f"{len(hbs)} MCU heartbeats after /clock stopped")
            self.assertFalse(
                [s for _, s in stacks if s.get("emit_mcu_heartbeat")],
                "stack still says emit_mcu_heartbeat after /clock stopped",
            )
            self.assertTrue(all(not nz for _, nz in self.cmds_since(t0 + lat)), "cmd_vel non-zero again")
        finally:
            state["run"] = False
            self.probe.destroy_timer(timer)
