#!/usr/bin/env python3
"""Sort the raw M8 records of the per-instance directories into the published files (ROS-free).

  python3 bench/fullstack/collect_m8.py [--src results/fullstack/m8] [--out results/fullstack] [--tag 2026-11-15_kitwip]

Reads <src>/<i>/faults.jsonl (fault trials) and <src>/<i>/sessions.jsonl (soak sessions) and writes, one line per record, unchanged:
  faults_m8_<tag>.jsonl                  label m8_faults (rule under test) and m8_faults_control (M7 watch set, same workload and trigger)
  soak_m8_<task>_<tag>.jsonl             label m8_soak, one file per task; label m8_reactive (item D, unloaded host) -> soak_m8_coverage_reactive_<tag>.jsonl
  m8_recon/sessions_recon.jsonl          label m8_recon (the stacks the recon traces were taken in)
  _m8_smoke/records_smoke.jsonl          label m8_smoke, and the first fault attempt with BT-triggered recoveries (task recovery_chain, label m8_faults:
                                         the wall scenario did not reach a spin at the nominal rate in 150 s; replaced by task recovery_probe_chain)
Records of other labels are written to _m8_smoke/records_other.jsonl so that nothing is dropped. Re-running overwrites the outputs.
"""

import argparse
import collections
import glob
import json
import os


def route(rec, mode, tag):
    """Output file name (relative to --out) for one record; `mode` is 'faults' or 'soak'."""
    label, task = rec.get("label") or "", rec.get("task")
    if label == "m8_recon":
        return "m8_recon/sessions_recon.jsonl"
    if label == "m8_smoke":
        return "_m8_smoke/records_smoke.jsonl"
    if mode == "soak" and label == "m8_reactive":
        return f"soak_m8_coverage_reactive_{tag}.jsonl"
    if mode == "faults" and label in ("m8_faults", "m8_faults_control"):
        if task == "recovery_chain":
            return "_m8_smoke/records_smoke.jsonl"
        return f"faults_m8_{tag}.jsonl"
    if mode == "soak" and label == "m8_soak":
        return f"soak_m8_{task}_{tag}.jsonl"
    return "_m8_smoke/records_other.jsonl"


def collect(src, out, tag):
    buckets = collections.defaultdict(list)
    for mode, name in (("faults", "faults.jsonl"), ("soak", "sessions.jsonl")):
        for path in sorted(glob.glob(os.path.join(src, "*", name))):
            for line in open(path):
                if line.strip():
                    buckets[route(json.loads(line), mode, tag)].append(line.rstrip("\n"))
    for rel, lines in buckets.items():
        path = os.path.join(out, rel)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            f.write("\n".join(lines) + "\n")
    return {k: len(v) for k, v in sorted(buckets.items())}


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.join(os.path.dirname(os.path.dirname(here)), "results", "fullstack")
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=os.path.join(root, "m8"))
    ap.add_argument("--out", default=root)
    ap.add_argument("--tag", default="2026-11-15_kitwip")
    a = ap.parse_args()
    for k, n in collect(a.src, a.out, a.tag).items():
        print(f"{n:4d} records -> {k}")


if __name__ == "__main__":
    main()
