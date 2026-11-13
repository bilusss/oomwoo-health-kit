"""The shipped example configs must stay valid and consistent with each other."""

import pathlib

import pytest

yaml = pytest.importorskip("yaml")

from oomwoo_health_kit.job_logic import parse_job_config  # noqa: E402
from oomwoo_health_kit.topic_watch import parse_watches  # noqa: E402

CONFIG = pathlib.Path(__file__).resolve().parent.parent / "config"
BASE_CRITICAL = {"lidar", "odom", "localization", "drive_supervision", "local_costmap"}


def load(name):
    return yaml.safe_load((CONFIG / name).read_text())


def watches():
    return {w.component_id: w for w in parse_watches(load("nav_watch.yaml"))}


def test_default_roster_is_static_and_critical_set_is_the_base():
    job = parse_job_config(load("nav_rosters.yaml"))
    assert job.source_type == "static"
    (rs,) = job.rosters.values()
    assert {c["component_id"] for c in rs.components if c["critical"]} == BASE_CRITICAL
    assert {c["component_id"] for c in rs.components if not c["critical"]} == {"global_costmap"}


def test_every_roster_component_has_a_watch_except_plumbing():
    w = watches()
    for f in ("nav_rosters.yaml", "nav_rosters_job.yaml"):
        for name, rs in parse_job_config(load(f)).rosters.items():
            for c in rs.components:
                assert c["component_id"] in w or c["component_id"] == "job_roster", f"{f}/{name}: {c['component_id']}"


def test_job_roster_is_optional_variant_with_fail_safe_shape():
    job = parse_job_config(load("nav_rosters_job.yaml"))
    assert job.strictest == "driving" and job.source_type == "nav2_actions"
    crit = lambda n: {c["component_id"] for c in job.rosters[n].components if c["critical"]}
    assert crit("driving") - crit("idle") == {"drive_path", "controller"}
    assert BASE_CRITICAL | {"job_roster"} <= crit("idle")


def test_watch_rules():
    w = watches()
    assert w["drive_path"].topic != "/cmd_vel"  # multi-writer topic
    assert w["drive_supervision"].is_motion
    for cid, spec in w.items():
        if not spec.is_tf and not spec.is_motion and cid != "controller":
            assert spec.max_publishers == 1, cid  # raising it must be deliberate


def test_nav_max_age_is_about_three_heartbeat_periods():
    w = watches()
    measured_hz = {"lidar": 5, "drive_path": 20, "controller": 5, "local_costmap": 5, "global_costmap": 5}
    job = parse_job_config(load("nav_rosters_job.yaml"))
    for c in job.rosters["driving"].components:
        hz = measured_hz.get(c["component_id"])
        if hz:
            period = 1.0 / min(hz, w[c["component_id"]].max_emit_hz)
            assert c["max_age_sec"] == pytest.approx(3 * period, rel=0.1), c["component_id"]
    odom = next(c for c in job.rosters["driving"].components if c["component_id"] == "odom")
    assert odom["max_age_sec"] >= 5 / w["odom"].max_emit_hz  # real margin over the heartbeat period


def test_message_types_resolve():
    pytest.importorskip("rosidl_runtime_py")
    from rosidl_runtime_py.utilities import get_message

    for w in watches().values():
        get_message(w.msg_type)


def test_sim_watch_config_covers_the_static_roster_without_stamp_checks():
    sim = {w.component_id: w for w in parse_watches(load("nav_watch_sim.yaml"))}
    (rs,) = parse_job_config(load("nav_rosters.yaml")).rosters.values()
    assert {c["component_id"] for c in rs.components} <= set(sim)
    # the sim watcher runs on wall time while header stamps are sim time: a stamp check would always say "degraded"
    assert all(w.stamp_max_age_s is None for w in sim.values())
    assert sim["drive_supervision"].motion["cmd_topic"] == "/cmd_vel"
    assert sim["localization"].is_tf and all(w.max_publishers == 1 for w in sim.values() if not w.is_tf and not w.is_motion)
