#!/usr/bin/env python3
"""Sort the raw M9 records of the per-instance directories into the published files (ROS-free).

  python3 bench/fullstack/collect_m9.py [--src results/fullstack/m9] [--out results/fullstack/m9/published]

Reads <src>/<i>/faults.jsonl and <src>/<i>/sessions.jsonl (raw, never modified) and writes copies, one line per record, unchanged:
  faults_m9.jsonl                  label m9_faults (any task)
  soak_m9_<task>.jsonl             label m9_soak (one file per task)
  recon_sessions.jsonl             labels m9_recon, m9_recon_gpu, m9_recon_retry (the stacks the beat traces were taken in; includes the invalid
                                   reactive start that crashed gz)
  other_records.jsonl              everything else (smoke runs, other labels), so that nothing is dropped.
Re-running overwrites the outputs.
"""

import argparse
import collections
import glob
import json
import os


def route(rec, mode):
    label, task = rec.get("label") or "", rec.get("task")
    if label.startswith("m9_recon"):
        return "recon_sessions.jsonl"
    if mode == "faults" and label == "m9_faults":
        return "faults_m9.jsonl"
    if mode == "soak" and label == "m9_soak":
        return f"soak_m9_{task}.jsonl"
    return "other_records.jsonl"


def collect(src, out):
    buckets = collections.defaultdict(list)
    for mode, name in (("faults", "faults.jsonl"), ("soak", "sessions.jsonl")):
        for path in sorted(glob.glob(os.path.join(src, "[0-9]*", name))):
            for line in open(path):
                if line.strip():
                    buckets[route(json.loads(line), mode)].append(line.rstrip("\n"))
    os.makedirs(out, exist_ok=True)
    for rel, lines in buckets.items():
        with open(os.path.join(out, rel), "w") as f:
            f.write("\n".join(lines) + "\n")
    return {k: len(v) for k, v in sorted(buckets.items())}


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    src = os.path.join(os.path.dirname(os.path.dirname(here)), "results", "fullstack", "m9")
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=src)
    ap.add_argument("--out", default=os.path.join(src, "published"))
    a = ap.parse_args()
    for k, n in collect(a.src, a.out).items():
        print(f"{n:4d} records -> {k}")


if __name__ == "__main__":
    main()
