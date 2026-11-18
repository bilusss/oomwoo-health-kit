#!/usr/bin/env bash
# M9: start bench/fullstack/beat_trace.py detached in ohk-sim-<i> (overlay environment), writing /results/<name>.jsonl (= results/fullstack/<OHK_RES_SUB>/<i>/).
# Usage: tools/m9_trace.sh <i> <seconds> <name> [beat_trace.py args]
set -euo pipefail
i="${1:?instance}"; secs="${2:?seconds}"; name="${3:?name}"; shift 3
n="ohk-sim-$i"
docker exec -d -u "$(id -u):$(id -g)" -e HOME=/tmp -e PYTHONDONTWRITEBYTECODE=1 "$n" bash -c \
  "source /ws/install/setup.bash; source /overlay/install/local_setup.bash; python3 -u /ws/src/oomwoo_health_kit/bench/fullstack/beat_trace.py --seconds $secs --out /results/$name.jsonl $* > /results/$name.out 2>&1"
echo "$n: beat_trace $secs s -> results/fullstack/${OHK_RES_SUB:-.}/$i/$name.jsonl"
