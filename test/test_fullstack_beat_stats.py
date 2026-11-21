"""ROS-free analysis of the M9 beat traces (bench/fullstack/beat_stats.py)."""

import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "bench" / "fullstack"))
import beat_stats as bs  # noqa: E402


def rows():
    r = [["odom", 0.0, 0.0, 0.0], ["state", 0.5, 0.5, "BACK"]]
    # beats every 0.5 s of stamp, one 1.5 s gap; robot moves from t=2.0
    stamps = [1.0, 1.5, 2.0, 2.5, 4.0, 4.5]
    for i, st in enumerate(stamps):
        r.append(["beat", st + 0.01, st + 0.03, "dock_detector", st, i + 1, "ok", ""])
    r.append(["odom", 2.2, 0.1, 0.0])
    return sorted(r, key=lambda x: x[1])


def test_percentile_nearest_rank():
    assert bs.pct([1, 2, 3, 4], 50) == 2 and bs.pct([1, 2, 3, 4], 100) == 4 and bs.pct([5], 99) == 5


def test_periods_ages_and_segments():
    a = bs.analyse(rows())["dock_detector"]
    allp = bs.summarize(a["all"])
    assert allp["periods_n"] == 5
    assert allp["period_stamp_s"]["max"] == 1.5 and allp["period_stamp_s"]["p50"] == 0.5
    assert abs(allp["age_at_receipt_s"]["max"] - 0.03) < 1e-9
    assert allp["seq_gaps"] == 0
    assert bs.summarize(a["moving"])["periods_n"] == 3          # beats at 2.5, 4.0, 4.5 are after the odom at 2.2
    assert bs.summarize(a["state_BACK"])["periods_n"] == 5


def test_sequence_gap_counted_and_files_are_not_bridged():
    r = rows()
    r = [x for x in r if not (x[0] == "beat" and x[5] == 3)]
    assert bs.analyse(r)["dock_detector"]["all"]["seq_gaps"] == 1
    m = bs.merge([bs.analyse(rows()), bs.analyse(rows())])
    assert len(m["dock_detector"]["all"]["stamp"]) == 10


def test_relay_moves_sim_age_onto_the_wall_clock_and_never_negative():
    import beat_relay as br
    p = {"component_id": "dock_drive", "health": "ok", "stamp_sec": 100.0, "sequence": 7, "detail": "x"}
    out = br.convert(p, sim_now=100.25, wall_now=1000.0)
    assert abs(out["stamp_sec"] - 999.75) < 1e-9
    assert {k: v for k, v in out.items() if k != "stamp_sec"} == {k: v for k, v in p.items() if k != "stamp_sec"}
    assert p["stamp_sec"] == 100.0                                   # input untouched
    assert br.convert(p, sim_now=99.999, wall_now=1000.0)["stamp_sec"] == 1000.0   # the relay's clock behind the sender's: age 0, not negative
    import pytest
    for bad in ({"component_id": "a"}, {"stamp_sec": "x"}, {"stamp_sec": float("nan")}, {"stamp_sec": float("inf")}):
        with pytest.raises((KeyError, ValueError, TypeError)):
            br.convert(bad, 1.0, 2.0)


def test_age_before_refresh_and_recommended_max_age():
    a = bs.analyse(rows())["dock_detector"]
    # stamps 1.0 1.5 2.0 2.5 4.0 4.5, receipt sim_now = stamp + 0.03: the 2.5 -> 4.0 pair: 4.03 - 2.5 = 1.53
    assert abs(max(a["all"]["abr"]) - 1.53) < 1e-9
    assert bs.recommended_max_age(a["all"]) == 2.3          # max(3 x 0.5, 1.5 x 1.53 = 2.295) -> 2.3
    flat = {"stamp": [0.6] * 9, "abr": [0.65] * 9}
    assert bs.recommended_max_age(flat) == 1.8              # 3 x 0.6 = 1.8 > 1.5 x 0.65


def test_collect_m9_routes_by_label_and_mode():
    import collect_m9 as c
    assert c.route({"label": "m9_faults", "task": "dock_inode"}, "faults") == "faults_m9.jsonl"
    assert c.route({"label": "m9_soak", "task": "dock_inode"}, "soak") == "soak_m9_dock_inode.jsonl"
    assert c.route({"label": "m9_recon_gpu", "task": "coverage_inode_recon"}, "soak") == "recon_sessions.jsonl"
    assert c.route({"label": "m9_smoke"}, "soak") == "other_records.jsonl"
    assert c.route({"label": "m9_soak"}, "faults") == "other_records.jsonl"


def test_relay_cannot_make_a_beat_fresher_or_invent_one():
    import json
    import beat_relay as br
    r = br.BeatRelay()
    sim0, wall0 = 500.0, 1.8e9
    msgs = [json.dumps({"component_id": "x", "health": "ok", "stamp_sec": sim0 + 0.2 * k, "sequence": k + 1, "detail": ""}) for k in range(10)]
    outs = []
    for k, m in enumerate(msgs):
        sim_now, wall_now = sim0 + 0.2 * k + 0.013 * (k % 3), wall0 + 0.2 * k + 0.013 * (k % 3)
        o = r.on_beat(m, sim_now, wall_now)
        assert o is not None
        j, p = json.loads(o), json.loads(m)
        # age the aggregator can compute from the output never undercuts the node's own age at receipt (wall = sim at RTF 1.0), and is >= 0
        assert wall_now - j["stamp_sec"] >= sim_now - p["stamp_sec"] - 1e-9 and wall_now - j["stamp_sec"] >= 0
        assert j["sequence"] == p["sequence"] and j["component_id"] == "x"
        outs.append(j["stamp_sec"])
    assert outs == sorted(outs) and len(set(outs)) == 10         # order kept, nothing duplicated
    assert r.n == 10 and r.bad == 0
    # a node that stopped: with no input there is no output at all (the class has no timer or stored beat); the aggregator's age only grows
    ages = [(wall0 + 5.0 + t) - outs[-1] for t in (0.0, 1.0, 2.0)]
    assert ages == sorted(ages) and ages[-1] > 5.0
    assert r.on_beat("not json", 1.0, 2.0) is None and r.on_beat('{"stamp_sec": "x"}', 1.0, 2.0) is None and r.bad == 2 and r.n == 10


def test_load_cuts_the_trace_where_the_next_stack_restarts_the_clock(tmp_path):
    import json
    f = tmp_path / "t.jsonl"
    rows = [["beat", 1.0, 10.0, "a", 10.0, 1, "ok", ""], ["beat", 1.2, 10.2, "a", 10.2, 2, "ok", ""], ["beat", 90.0, 3.0, "a", 3.0, 1, "ok", ""]]
    f.write_text("\n".join(json.dumps(r) for r in rows))
    assert [r[5] for r in bs.load(str(f))] == [1, 2]


def test_collect_m9b_routes():
    import collect_m9 as c
    assert c.route({"label": "m9b_faults", "task": "dock_inode"}, "faults") == "faults_m9b.jsonl"
    assert c.route({"label": "m9b_soak", "task": "coverage_inode"}, "soak") == "soak_m9b_coverage_inode.jsonl"
    assert c.route({"label": "m9b_smoke"}, "soak") == "other_records.jsonl"
