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
        assert {c["component_id"] for c in rs.components} <= w, name
        assert "drive_supervision" in {c["component_id"] for c in rs.components if c["critical"]}, name


def test_amcl_global_costmap_max_age_is_three_periods_of_2hz():
    (rs,) = parse_job_config(yaml.safe_load((ROOT / "config" / "nav_rosters_amcl.yaml").read_text())).rosters.values()
    c = {x["component_id"]: x for x in rs.components}
    assert c["global_costmap"]["max_age_sec"] >= 3 * 0.5 and not c["global_costmap"]["critical"]
    assert c["odom"]["critical"] and c["localization"]["critical"]


def test_launch_args():
    a = tk.launch_args("dock", "false")
    assert "localization:=none" in a and "job:=dock" in a and "world:=kitchen_dining.world" in a and "composition:=false" in a


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
        assert (sig == "SIGSTOP") == resume and when in ("moving", "fast", "recovery", "dock_back"), name


def test_reactive_variant_differs_only_in_cmd_max_age():
    def load(n):
        return yaml.safe_load((ROOT / "config" / n).read_text())
    a, b = load("nav_watch_amcl.yaml"), load("nav_watch_amcl_reactive.yaml")
    mc = lambda d: [w for w in d["watches"] if w["component_id"] == "drive_supervision"][0]["motion_consistency"]  # noqa: E731
    assert mc(a)["cmd_max_age_s"] == 0.15 and mc(b)["cmd_max_age_s"] == 0.30
    mc(a)["cmd_max_age_s"] = 0.30
    assert a == b
