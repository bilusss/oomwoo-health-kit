"""Optional job roster, nav2_actions source, one task_id and identical ids (only critical flips)."""

import _o_common as c
import ohk_nav_harness as n


def generate_test_description():
    return n.build_nav_description(
        n.job_rosters(constant_ids=True), fakes=("scan", "odom", "tf", "polygon", "drive"),
        odom_extra=("--vx", "0.2", "--enable-topic", n.ENABLE_TOPIC),
    )


class TestJobConstantIds(c.JobTransitions):
    NAME = "o2_job_constant_ids"
