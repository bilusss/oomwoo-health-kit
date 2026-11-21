"""ROS-free parts of the M7 bench (bench/fullstack/tasks.py) and the M7 watcher configs."""

import math
import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "bench" / "fullstack"))
import tasks as tk  # noqa: E402

yaml = pytest.importorskip("yaml")
from oomwoo_health_kit.job_logic import parse_job_config  # noqa: E402
from oomwoo_health_kit.topic_watch import parse_watches  # noqa: E402


def test_every_task_has_existing_configs_that_parse_and_match():
    for name, t in tk.TASKS.items():
        w = {x.component_id for x in parse_watches(yaml.safe_load((ROOT / "config" / t["watch"]).read_text()))}
        (rs,) = parse_job_config(yaml.safe_load((ROOT / "config" / t["roster"]).read_text())).rosters.values()
        # M9: in-node components are published by the nodes themselves, not by the watcher
        inode = set(t.get("inode", ()))
        assert {c["component_id"] for c in rs.components} - inode <= w, name
        assert not inode & w, name
        assert inode <= {c["component_id"] for c in rs.components if c["critical"]}, name
        assert "drive_supervision" in {c["component_id"] for c in rs.components if c["critical"]}, name


def test_amcl_global_costmap_max_age_is_three_periods_of_2hz():
    (rs,) = parse_job_config(yaml.safe_load((ROOT / "config" / "nav_rosters_amcl.yaml").read_text())).rosters.values()
    c = {x["component_id"]: x for x in rs.components}
    assert c["global_costmap"]["max_age_sec"] >= 3 * 0.5 and not c["global_costmap"]["critical"]
    assert c["odom"]["critical"] and c["localization"]["critical"]


def test_launch_args():
    a = tk.launch_args("dock", "false")
    assert "localization:=none" in a and "job:=dock" in a and "world:=kitchen_dining.world" in a and "composition:=false" in a


def test_inode_tasks_select_inode_launch_arg_and_other_tasks_do_not():
    assert "inode_health:=true" in tk.launch_args("dock_inode", "true")
    assert "inode_health:=true" in tk.launch_args("coverage_reactive_inode_recon", "true")
    for name, t in tk.TASKS.items():
        assert ("inode_health:=true" in tk.launch_args(name)) == bool(t.get("inode")), name


def test_beat_relay_is_opt_in_and_reactive_roster_has_margin():
    assert "beat_relay:=false" in tk.launch_args("dock_inode") and "beat_relay:=true" in tk.launch_args("dock_inode", "true", True)
    yaml_ = pytest.importorskip("yaml")
    from oomwoo_health_kit.job_logic import parse_job_config
    for task, want in (("coverage_inode", 0.6), ("coverage_reactive_inode", 0.8), ("dock_inode", None)):
        (rs,) = parse_job_config(yaml_.safe_load((ROOT / "config" / tk.TASKS[task]["roster"]).read_text())).rosters.values()
        c = {x["component_id"]: x["max_age_sec"] for x in rs.components}
        if want:
            assert c["coverage_planner"] == want, task
        else:
            assert c["dock_detector"] == 1.8 and c["dock_drive"] == 0.15
        assert not any("coverage_planner" in x["component_id"] and x["max_age_sec"] < 0.6 for x in rs.components)


def test_planner_cmd_health_component_id_is_opt_in():
    sys.path.insert(0, str(ROOT / "bench"))
    sys.path.insert(0, str(ROOT / "bench" / "fullstack"))
    pytest.importorskip("rclpy", reason="jobs.py imports ohk_bench")
    import jobs
    assert not any("health_component_id" in a for a in jobs.planner_cmd("nav2", prefix="/x"))
    assert "health_component_id:=coverage_planner" in jobs.planner_cmd("reactive", prefix="/x", component_id="coverage_planner")


def test_wall_sdf():
    assert "<size>2.4 0.05 1</size>" in tk.wall_sdf("w") and 'model name="w"' in tk.wall_sdf("w")


def test_wall_ahead_is_across_the_line_to_the_goal():
    x, y, yaw = tk.wall_ahead((0, 0), (2, 0), 0.7)
    assert (round(x, 3), round(y, 3)) == (0.7, 0.0) and round(yaw, 3) == round(math.pi / 2, 3)


def test_action_tally_counts_each_goal_once():
    t = tk.ActionTally()
    t.update(1.0, "spin", [("a", 2)])
    t.update(1.1, "spin", [("a", 2)])
    t.update(2.0, "spin", [("a", 4)])
    t.update(2.1, "spin", [("a", 4), ("b", 4)])      # b first seen terminal (latched): not counted
    t.update(3.0, "spin", [("a", 4), ("c", 1), ("b", 4)])
    assert t.counts["spin"] == {"started": 2, "succeeded": 1, "canceled": 0, "aborted": 0}
    assert t.started_between(0, 2.5) == {"spin": 1} and t.started_between(0, 9, names=("wait",)) == {}


def test_planner_log_parser():
    lines = ["[INFO] coverage plan: 234 waypoints, executing sequentially", "waypoint 10/234, coverage 4.0%",
             "waypoint 16 stuck (no progress); skipping", "waypoint 16 unreachable after 2 tries; skipping", "waypoint 20/234, coverage 4.3%"]
    r = tk.parse_planner_log(lines)
    assert r == {"waypoints": 234, "last_waypoint": 20, "coverage_pct": 4.3, "stuck": 1, "unreachable": 1, "complete": False}


def test_planner_log_complete_flag():
    assert tk.parse_planner_log(["[INFO] [coverage_planner]: waypoint 140/234, coverage 67.1%", "coverage complete: target hit"])["complete"] is True
    assert tk.parse_planner_log(["Coverage Complete"])["complete"] is True


def test_when_ok():
    assert tk.when_ok("moving", 0.2, 0) and not tk.when_ok("moving", 0.1, 5)
    assert tk.when_ok("fast", 0.35, 0) and not tk.when_ok("fast", 0.2, 0)          # Nav2 transits (0.2 m/s) are not reactive passes
    assert tk.when_ok("recovery", 0, 0.5, recovery_active=True) and not tk.when_ok("recovery", 0, 0.5, recovery_active=False)
    assert not tk.when_ok("recovery", 0.0, 0.0, recovery_active=True)               # a wait: active but not moving
    assert tk.when_ok("dock_back", 0.06, 0, dock_state="BACK") and not tk.when_ok("dock_back", 0.06, 0, dock_state="TURN")
    assert tk.when_ok("unknown", 0.2, 0)


def test_gz_client_builds_requests_without_gz(monkeypatch):
    """Jobs.Gz with fake gz modules: service names, request fields, reply handling."""
    import types
    calls = []

    class FakeNode:
        def request(self, service, req, rq, rp, timeout):
            calls.append((service, req))
            return True, types.SimpleNamespace(data=True)

    def ns(**kw):
        return types.SimpleNamespace(**kw)

    class Msg:
        def __init__(self):
            self.position = ns(x=0, y=0, z=0)
            self.orientation = ns(z=0, w=1)
            self.pose = ns(position=self.position, orientation=self.orientation)

    class Entity(Msg):
        MODEL = 2

    mods = {"gz": types.ModuleType("gz"), "gz.transport13": ns(Node=FakeNode), "gz.msgs10": types.ModuleType("gz.msgs10"),
            "gz.msgs10.boolean_pb2": ns(Boolean=Msg), "gz.msgs10.pose_pb2": ns(Pose=Msg),
            "gz.msgs10.entity_factory_pb2": ns(EntityFactory=Msg), "gz.msgs10.entity_pb2": ns(Entity=Entity)}
    for k, v in mods.items():
        monkeypatch.setitem(sys.modules, k, v)
    monkeypatch.setitem(sys.modules, "ohk_bench", ns(die_with_parent=lambda: None))
    monkeypatch.setitem(sys.modules, "workload", ns(POSES=[], Workload=object))
    sys.modules.pop("jobs", None)
    import jobs
    g = jobs.Gz("w")
    assert g.set_pose("m", 1.0, 2.0, 0.06, math.pi) is True
    svc, req = calls[-1]
    assert svc == "/world/w/set_pose" and req.name == "m" and (req.position.x, req.position.y) == (1.0, 2.0)
    assert abs(req.orientation.z - 1.0) < 1e-9 and abs(req.orientation.w) < 1e-9
    assert g.create_box("b", "/tmp/b.sdf", 0.5, 0.6, 0.0) is True
    svc, req = calls[-1]
    assert svc == "/world/w/create" and req.sdf_filename == "/tmp/b.sdf" and req.name == "b" and req.pose.position.x == 0.5
    assert g.remove("b") is True
    svc, req = calls[-1]
    assert svc == "/world/w/remove" and req.name == "b" and req.type == Entity.MODEL
    sys.modules.pop("jobs", None)


def test_m7_faults_cover_the_commanding_nodes():
    assert {"sigkill_behavior", "sigkill_dock_drive", "sigkill_coverage", "sigkill_amcl"} <= set(tk.M7_FAULTS)
    for name, (frag, sig, resume, when) in tk.M7_FAULTS.items():
        assert (sig == "SIGSTOP") == resume and when in ("moving", "fast", "recovery", "recovery_nominal", "dock_back"), name


def test_reactive_variant_differs_only_in_cmd_max_age():
    def load(n):
        return yaml.safe_load((ROOT / "config" / n).read_text())
    a, b = load("nav_watch_amcl.yaml"), load("nav_watch_amcl_reactive.yaml")
    mc = lambda d: [w for w in d["watches"] if w["component_id"] == "drive_supervision"][0]["motion_consistency"]  # noqa: E731
    assert mc(a)["cmd_max_age_s"] == 0.15 and mc(b)["cmd_max_age_s"] == 0.30
    mc(a)["cmd_max_age_s"] = 0.30
    assert a == b


def test_m8_tasks_select_new_rules_and_leave_the_m7_tasks_alone():
    for name, base in (("nav_amcl_chain", "nav_amcl"), ("recovery_chain", "recovery"), ("dock_fix", "dock"), ("recovery_probe_chain", "recovery_probe")):
        a, b = dict(tk.TASKS[name]), dict(tk.TASKS[base])
        for k in ("watch", "roster"):
            assert a.pop(k) != b.pop(k)
        assert a == b, name
    assert tk.TASKS["nav_amcl"]["watch"] == "nav_watch_amcl.yaml" and tk.TASKS["dock"]["watch"] == "nav_watch_dock.yaml"


def test_recovery_nominal_trigger_matches_the_exposure_criterion():
    ok = lambda sp, w, act=True: tk.when_ok("recovery_nominal", sp, w, recovery_active=act)  # noqa: E731
    assert ok(0.0, 0.95) and ok(0.12, 0.0)                  # nominal spin, nominal backup
    assert not ok(0.0, 0.71) and not ok(0.05, 0.2)          # tail of a spin, a creeping backup
    assert not ok(0.0, 0.95, act=False) and not ok(0.0, 0.0)  # no action active, a wait
    assert tk.M7_FAULTS["sigkill_behavior_nominal"][3] == "recovery_nominal"


def _watch_doc(n):
    return yaml.safe_load((ROOT / "config" / n).read_text())


def test_chain_config_is_the_amcl_config_plus_one_watch_and_one_critical_component():
    a, b = _watch_doc("nav_watch_amcl.yaml"), _watch_doc("nav_watch_amcl_chain.yaml")
    extra = [w for w in b["watches"] if w not in a["watches"]]
    assert [w["component_id"] for w in extra] == ["command_chain"] and all(w in b["watches"] for w in a["watches"])
    (c,) = [w for w in parse_watches(b) if w.is_chain]
    assert (c.chain["in_topic"], c.chain["out_topic"]) == ("/cmd_vel_nav", "/cmd_vel_smoothed")
    # bounds derived in docs/M8_RECON.md: 1.5 x the measured maxima, rounded up to 50 ms
    assert c.chain["in_max_age_s"] == pytest.approx(math.ceil(1.5 * 0.201 / 0.05) * 0.05)
    assert c.chain["stop_grace_s"] == pytest.approx(math.ceil(1.5 * 0.344 / 0.05) * 0.05)
    ra = parse_job_config(_watch_doc("nav_rosters_amcl.yaml")).rosters["nav"].components
    rb = parse_job_config(_watch_doc("nav_rosters_amcl_chain.yaml")).rosters["nav"].components
    assert [x for x in rb if x not in ra] == [{"component_id": "command_chain", "critical": True, "max_age_sec": 0.15}]
    assert all(x in rb for x in ra)


def test_m7_configs_do_not_use_the_m8_rules():
    for n in ("nav_watch.yaml", "nav_watch_sim.yaml", "nav_watch_amcl.yaml", "nav_watch_amcl_reactive.yaml", "nav_watch_dock.yaml"):
        assert not any(w.is_chain or w.is_reverse_fix for w in parse_watches(_watch_doc(n))), n


def test_dock_fix_config_is_the_dock_config_plus_one_watch_with_the_derived_bound():
    a, b = _watch_doc("nav_watch_dock.yaml"), _watch_doc("nav_watch_dock_fix.yaml")
    extra = [w for w in b["watches"] if w not in a["watches"]]
    assert [w["component_id"] for w in extra] == ["dock_fix"] and all(w in b["watches"] for w in a["watches"])
    (r,) = [w for w in parse_watches(b) if w.is_reverse_fix]
    # recon: p99 of the gaps between consecutive fixes inside reversing episodes 1.202 s; bound = ceil_50ms(1.5 x p99)
    assert r.reverse_fix["fix_max_age_s"] == pytest.approx(math.ceil(1.5 * 1.202 / 0.05) * 0.05)
    assert r.reverse_fix["fix_max_age_s"] < 3.0 and r.reverse_fix["from_reverse_start"] is False   # dock_drive's own pose_timeout_s is 3.0
    ra = parse_job_config(_watch_doc("nav_rosters_dock.yaml")).rosters["nav"].components
    rb = parse_job_config(_watch_doc("nav_rosters_dock_fix.yaml")).rosters["nav"].components
    assert [x for x in rb if x not in ra] == [{"component_id": "dock_fix", "critical": True, "max_age_sec": 0.15}] and all(x in rb for x in ra)


def test_only_the_dock_config_selects_rule_b():
    """Rule B treats ANY writer that reverses on /cmd_vel as the dock commander: it may only be used in the dock stack."""
    users = [p.name for p in sorted((ROOT / "config").glob("nav_watch*.yaml")) if any(w.is_reverse_fix for w in parse_watches(yaml.safe_load(p.read_text())))]
    assert users == ["nav_watch_dock_fix.yaml"]
    assert [t for t, v in tk.TASKS.items() if v["watch"] == "nav_watch_dock_fix.yaml"] == ["dock_fix"] and tk.TASKS["dock_fix"]["nav"] is False
