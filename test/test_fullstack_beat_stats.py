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
