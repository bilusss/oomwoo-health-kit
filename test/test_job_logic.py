import pytest

from oomwoo_health_kit.job_logic import Debouncer, StatusSource, any_active, parse_job_config

RANKS = {"idle": 0.0, "driving": 1.0}


def test_any_active():
    assert any_active([1]) and any_active([4, 2]) and any_active([3])
    assert not any_active([]) and not any_active([0, 4, 5, 6])


def test_debouncer_stricter_is_immediate():
    d = Debouncer("idle", 0.3, RANKS)
    assert d.observe(0.0, "driving") == "driving" and d.committed == "driving"


def test_debouncer_relaxation_waits_for_hold():
    d = Debouncer("driving", 0.3, RANKS)
    assert d.observe(0.0, "idle") is None
    assert d.poll(0.29) is None and d.committed == "driving"
    assert d.poll(0.3) == "idle" and d.committed == "idle"
    assert d.poll(0.5) is None


def test_debouncer_flap_does_not_relax():
    d = Debouncer("driving", 0.3, RANKS)
    d.observe(0.0, "idle")
    d.observe(0.2, "driving")  # next goal before the hold ended
    assert d.poll(1.0) is None and d.committed == "driving"


def test_debouncer_repeat_observation_keeps_hold_start():
    d = Debouncer("driving", 0.3, RANKS)
    d.observe(0.0, "idle")
    d.observe(0.2, "idle")
    assert d.poll(0.31) == "idle"


def test_debouncer_clock_backwards_selects_the_strictest_set():
    d = Debouncer("driving", 0.3, RANKS)
    d.observe(10.0, "idle")  # relaxation pending
    assert d.poll(2.0) is None and d.committed == "driving"  # jump back: NOT a relaxation
    assert d.poll(2.4) is None and d.committed == "driving"  # and the pending relaxation was dropped
    d = Debouncer("idle", 0.3, RANKS)
    d.observe(10.0, "idle")
    assert d.observe(3.0, "idle") == "driving" and d.committed == "driving"  # relaxed state + jump -> strictest
    d = Debouncer("idle", 0.3, RANKS)
    d.observe(10.0, "idle")
    assert d.poll(1.0) == "driving"


def test_status_source_unknown_until_known():
    s = StatusSource(["/a", "/b"], settle_s=1.0)
    assert s.state(0.0) == "unknown"
    s.on_publishers(0.0, "/a", 1)
    s.on_publishers(0.0, "/b", 1)
    assert s.state(0.5) == "unknown"  # latched value could still arrive
    assert s.state(1.0) == "inactive"  # publishers present for settle_s and nothing latched: no goal ever
    s.on_status("/a", True)
    assert s.state(1.0) == "active"
    s.on_status("/a", False)
    assert s.state(1.0) == "inactive"


def test_status_source_message_makes_topic_known_and_missing_publisher_is_invalid():
    s = StatusSource(["/a"], settle_s=1.0)
    s.on_status("/a", False)
    assert s.state(0.0) == "inactive"
    assert not s.valid()  # no publisher seen yet
    s.on_publishers(0.0, "/a", 1)
    assert s.valid()
    s.on_publishers(1.0, "/a", 0)
    assert not s.valid()


def comp(cid, critical=True, age=0.5):
    return {"component_id": cid, "critical": critical, "max_age_sec": age}


def cfg(**over):
    c = {
        "debounce_s": 0.3,
        "state_source": {"type": "topic"},
        "rosters": {
            "idle": {"task_id": "nav", "components": [comp("a"), comp("b", False, 1.0)]},
            "driving": {"task_id": "nav", "components": [comp("a"), comp("b", True, 1.0)]},
        },
    }
    c.update(over)
    return c


def test_parse_strictest_ranks_and_plumbing_component():
    c = parse_job_config(cfg())
    assert c.strictest == "driving" and c.ranks == {"idle": 0.0, "driving": 1.0}
    for rs in c.rosters.values():
        j = [x for x in rs.components if x["component_id"] == "job_roster"]
        assert j == [{"component_id": "job_roster", "critical": True, "max_age_sec": 0.5}]
    p = c.rosters["driving"].payload()
    assert p["task_id"] == "nav" and p["components"][1] == {"component_id": "b", "critical": True, "max_age_sec": 1.0}
    assert c.source_type == "topic" and c.job_state_topic == "oomwoo/health/job_state"


def test_parse_strictest_by_tighter_max_age_when_counts_tie():
    d = cfg(rosters={
        "loose": {"task_id": "n", "components": [comp("a", True, 1.0)]},
        "tight": {"task_id": "n", "components": [comp("a", True, 0.2)]},
    })
    assert parse_job_config(d).strictest == "tight"


def test_parse_static_has_no_plumbing_component():
    c = parse_job_config({"state_source": {"type": "static"}, "rosters": {"nav": {"task_id": "nav", "components": [comp("a")]}}})
    assert [x["component_id"] for x in c.rosters["nav"].components] == ["a"] and c.strictest == "nav"


def test_parse_nav2_source():
    c = parse_job_config(cfg(state_source={"type": "nav2_actions", "status_topics": ["/a/_action/status"]}))
    assert c.status_topics == ("/a/_action/status",) and (c.active_state, c.inactive_state) == ("driving", "idle")


def test_parse_refuses_differing_ids_or_task_ids_unless_allowed():
    ids = cfg(rosters={"idle": {"task_id": "nav", "components": [comp("a")]},
                       "driving": {"task_id": "nav", "components": [comp("a"), comp("b")]}})
    tasks = cfg(rosters={"idle": {"task_id": "x", "components": [comp("a")]},
                         "driving": {"task_id": "y", "components": [comp("a")]}})
    for bad in (ids, tasks):
        with pytest.raises(ValueError, match="identical"):
            parse_job_config(bad)
        parse_job_config({**bad, "allow_arming_restart": True})


@pytest.mark.parametrize(
    "bad",
    [
        None,
        cfg(rosters={}),
        cfg(initial_state="idle"),  # must be the strictest set
        cfg(debounce_s=-1),
        cfg(state_source={"type": "magic"}),
        cfg(state_source={"type": "static"}),  # two sets
        cfg(state_source={"type": "nav2_actions"}),
        cfg(state_source={"type": "nav2_actions", "status_topics": ["/x"], "active_state": "nope"}),
        cfg(allow_arming_restart="yes"),
        cfg(rosters={"idle": {"task_id": "", "components": [comp("a")]}}),
        cfg(rosters={"idle": {"task_id": "t", "components": []}}),
        cfg(rosters={"idle": {"task_id": "t", "components": [{"component_id": "a"}]}}),
        cfg(rosters={"idle": {"task_id": "t", "components": [comp("a", True, 0)]}}),
        cfg(rosters={"idle": {"task_id": "t", "components": [{**comp("a"), "critical": "yes"}]}}),
        cfg(rosters={"idle": {"task_id": "t", "components": [comp("a")] * 2}}),
        cfg(rosters={"idle": {"task_id": "t", "components": [comp("job_roster", False)]}}),
    ],
)
def test_parse_rejects(bad):
    with pytest.raises(ValueError):
        parse_job_config(bad)
