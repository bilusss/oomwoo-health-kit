"""Optional job roster, nav2_actions source, task_id/ids differ between the sets (allow_arming_restart)."""

import _o_common as c
import ohk_nav_harness as n


def generate_test_description():
    return n.build_nav_description(
        n.job_rosters(constant_ids=False), fakes=("scan", "odom", "tf", "polygon", "drive"),
        odom_extra=("--vx", "0.2", "--enable-topic", n.ENABLE_TOPIC),
    )


class TestJobTransition(c.JobTransitions):
    NAME = "o_job_transition"
