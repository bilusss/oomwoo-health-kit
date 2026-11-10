import json

import pytest

from oomwoo_health_kit.heartbeat import HEALTH_LEVELS, WorkHeartbeat, to_json

FIELDS = {"component_id", "health", "stamp_sec", "sequence", "detail"}


def test_payload_schema_matches_contract():
    payload = WorkHeartbeat("recovery_safety").beat(12.5)
    assert set(payload) == FIELDS
    assert payload["component_id"] == "recovery_safety"
    assert payload["health"] == "ok"
    assert payload["stamp_sec"] == 12.5
    assert payload["detail"] == ""
    assert isinstance(payload["sequence"], int)


def test_level_vocabulary():
    assert HEALTH_LEVELS == {"ok", "healthy", "warn", "degraded", "error"}
    hb = WorkHeartbeat("c")
    for level in HEALTH_LEVELS:
        assert hb.beat(0.0, level)["health"] == level


def test_unknown_level_rejected_without_consuming_sequence():
    hb = WorkHeartbeat("c")
    with pytest.raises(ValueError):
        hb.beat(0.0, "fine")
    assert hb.beat(0.0)["sequence"] == 1


def test_empty_component_id_rejected():
    with pytest.raises(ValueError):
        WorkHeartbeat("")


def test_sequence_starts_at_one_and_increments_by_one():
    hb = WorkHeartbeat("c")
    assert [hb.beat(float(i))["sequence"] for i in range(5)] == [1, 2, 3, 4, 5]


def test_sequences_are_per_instance():
    a, b = WorkHeartbeat("a"), WorkHeartbeat("b")
    a.beat(0.0)
    a.beat(0.1)
    assert b.beat(0.0)["sequence"] == 1


def test_detail_passthrough():
    assert WorkHeartbeat("c").beat(1.0, "warn", "slow scan")["detail"] == "slow scan"


def test_json_round_trip():
    payload = WorkHeartbeat("loc").beat(3.25, "degraded", "x")
    text = to_json(payload)
    assert isinstance(text, str)
    assert json.loads(text) == payload


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
def test_non_finite_stamp_rejected_without_consuming_sequence(bad):
    hb = WorkHeartbeat("c")
    with pytest.raises(ValueError):
        hb.beat(bad)
    assert hb.beat(0.0)["sequence"] == 1
