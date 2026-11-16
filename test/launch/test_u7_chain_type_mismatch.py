"""M8 review fix: a rule that watches the wrong message type must not switch itself off silently. The watch is configured for TwistStamped (the
newer Nav2 default) while the stack publishes Twist: the component must be not-ok from the start, so the aggregator never reports a healthy stack
and the stand-in does not run, with the reason in the watcher's log text (graph check)."""

import time

import ohk_nav_harness as n


def generate_test_description():
    return n.m8_description("chain", fakes=("scan", "odom", "tf", "polygon", "smoother", "commander"),
                            chain_overrides={"msg_type": "geometry_msgs/msg/TwistStamped"})


class TestChainTypeMismatch(n.NavCase):
    PROBE_FEED = False

    def test_wrong_message_type_is_not_healthy(self):
        t0 = time.monotonic()
        time.sleep(6.0)
        stacks = [s for _, s in self.stacks_since(t0)]
        self.assertTrue(stacks, "the aggregator published")
        self.assertFalse(any(s["state"].startswith("healthy") for s in stacks), "never healthy with a mistyped watch")
        self.assertTrue(any("command_chain" in (s["stale_critical"] + s["missing_critical"] + s["unhealthy_critical"]) for s in stacks))
        self.assertNotEqual(self.current_state(), "run")
        self.record("u7_chain_type_mismatch", stack_msgs=len(stacks))
