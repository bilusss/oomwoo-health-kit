"""job_roster (optional) with the String state source: starts strictest, stricter at once, relaxation debounced,
unknown state -> strictest, job_roster heartbeat present in every set."""

import time

import ohk_nav_harness as n


def generate_test_description():
    return n.build_nav_description(n.job_rosters(source="topic"))


class TestJobStateTopic(n.NavCase):
    def drive_critical(self):
        return self.crit(self.last_roster(), "drive_path")

    def test_follows_topic_fail_safe(self):
        # Starts strictest: drive_path critical and silent -> fault, robot stopped.
        self.wait_for(lambda: self.last_roster() is not None, 5.0, "first roster")
        self.assertTrue(self.drive_critical(), "first roster must be the strictest set")
        self.assertIsNotNone(self.crit(self.last_roster(), "job_roster"))
        t0 = self.set_job_state("idle")
        self.assertTrue(self.drive_critical(), "relaxation must be debounced")
        self.wait_for(lambda: self.drive_critical() is False, 3.0, "relaxed roster")
        self.assertGreaterEqual(self.rosters[-1][0] - t0, 0.29)
        self.wait_idle_run()
        t1 = self.set_job_state("driving")  # stricter: at once (drive_path silent -> stack faults)
        self.wait_for(lambda: self.drive_critical() is True, 0.5, "strict roster at once")
        self.assertLess(self.rosters[-1][0] - t1, 0.2)
        self.set_job_state("idle")
        self.wait_for(lambda: self.drive_critical() is False, 3.0, "relaxed again")
        t2 = self.set_job_state("flying")  # unknown -> strictest
        self.wait_for(lambda: self.drive_critical() is True, 0.5, "unknown state selects strictest")
        time.sleep(0.8)
        self.assertTrue(self.drive_critical())
        self.record("q_job_state_topic", ok=True, strict_after_unknown_ms=round((self.rosters[-1][0] - t2) * 1000))
