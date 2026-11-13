#!/usr/bin/env bash
# Fleet of headless full-stack OOMWOO sim containers (ohk-sim-1 .. ohk-sim-N) for bench/fullstack/run.py.
# Usage: tools/sim.sh {build|up N [--gpu]|run <i|all> [--fg] [run.py args]|stop <i|all>|status|down [--purge]|exec <i> CMD...}
#
#   build            clone xbattlax/health-monitor at OHK_AGG_REF (default b2b9796) and colcon-build it plus this kit into the
#                    shared volume ohk-sim-ws (one throwaway container ohk-sim-build; the sim containers mount it read-only).
#   up N [--gpu]     (re)create N containers from makerspet/oomwoo:jazzy-dev; each has its own network namespace and its own
#                    ROS_DOMAIN_ID (40+i) and GZ_PARTITION, --shm-size=2g, the project read-only at /ws/src/oomwoo_health_kit,
#                    results/fullstack/<i> read-write at /results. Containers run as the host uid (no root files in the project).
#                    --gpu adds --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all and OHK_GPU=1 (EGL vendor file + non-llvmpipe launch,
#                    see bench/fullstack/stack.launch.py). A container with a different GPU setting is recreated.
#   run <i|all> ...  start bench/fullstack/run.py DETACHED in that container (docker exec -d: survives this shell); stdout in
#                    results/fullstack/<i>/run.out. With --fg: run in the foreground. Refuses if a run is already active there.
#   stop <i|all>     SIGTERM the run (it stops the stack cleanly and writes the session line), wait 90 s, then SIGKILL.
#   status           containers, active runs, progress of each, last JSONL line per instance, container CPU.
#   down [--purge]   remove the ohk-sim-* containers (--purge also removes the ohk-sim-ws build volume).
# Only containers named ohk-sim-* / ohk-sim-build are ever touched; ohk-dev and everything else is left alone.
set -euo pipefail
IMAGE=makerspet/oomwoo:jazzy-dev
VOL=ohk-sim-ws
AGG_URL=https://github.com/xbattlax/health-monitor.git
AGG_COMMIT="${OHK_AGG_REF:-b2b9796}"
PROJECT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RES="$PROJECT/results/fullstack"
UIDGID="$(id -u):$(id -g)"
WS_ENV='source /ws/install/setup.bash'

names() { docker ps -a --filter 'name=^ohk-sim-[0-9]+$' --format '{{.Names}}' | sort -t- -k3 -n; }
want() {  # <i|all> -> container names
  if [ "$1" = all ]; then names; else echo "ohk-sim-$1"; fi
}
xrun() {  # exec as host user in container $1: xrun NAME cmd...
  local n="$1"; shift
  docker exec -u "$UIDGID" -e HOME=/tmp -e PYTHONDONTWRITEBYTECODE=1 -w /ws "$n" bash -c "$WS_ENV && $*"
}

build() {
  docker volume create "$VOL" >/dev/null
  docker run --rm -u 0 -v "$VOL:/ws" "$IMAGE" bash -c "mkdir -p /ws/src/oomwoo_health_kit && chown $UIDGID /ws /ws/src" 2>&1 | grep -v 'Visit https' || true
  docker rm -f ohk-sim-build >/dev/null 2>&1 || true
  docker run --rm --name ohk-sim-build --init -u "$UIDGID" -e HOME=/tmp -e PYTHONDONTWRITEBYTECODE=1 -w /ws \
    -v "$VOL:/ws" -v "$PROJECT:/ws/src/oomwoo_health_kit:ro" "$IMAGE" bash -c '
      set -e
      [ -d /ws/src/health-monitor/.git ] || git clone -q '"$AGG_URL"' /ws/src/health-monitor
      cd /ws/src/health-monitor
      git rev-parse -q --verify "'"$AGG_COMMIT"'^{commit}" >/dev/null 2>&1 || git fetch -q origin "+refs/heads/*:refs/remotes/origin/*" "+refs/pull/*/head:refs/remotes/origin/pr/*"
      git checkout -q '"$AGG_COMMIT"'
      want=$(git rev-parse HEAD); have=$(cat /ws/.agg_built 2>/dev/null || true)
      echo "aggregator ref '"$AGG_COMMIT"' ($want)"
      [ "$want" = "$have" ] || rm -rf /ws/build/oomwoo_health_monitor /ws/install/oomwoo_health_monitor
      cd /ws
      colcon build --parallel-workers 6 --event-handlers console_direct+
      echo "$want" > /ws/.agg_built
      echo "image: '"$IMAGE"'" > /ws/.build_info' 2>&1 | grep -v 'Visit https'
  echo "build done: aggregator $(docker run --rm -v "$VOL:/ws:ro" "$IMAGE" cat /ws/.agg_built 2>/dev/null | tail -1)"
}

up() {
  local n="${1:?usage: up N [--gpu]}"; shift || true
  local gpu=0
  [ "${1:-}" = "--gpu" ] && gpu=1
  docker run --rm -v "$VOL:/ws:ro" "$IMAGE" test -f /ws/install/setup.bash 2>/dev/null \
    || { echo "no build in volume $VOL; run: tools/sim.sh build" >&2; exit 3; }
  for i in $(seq "$n"); do
    local name="ohk-sim-$i" cur=""
    mkdir -p "$RES/$i"
    if docker container inspect "$name" >/dev/null 2>&1; then
      cur="$(docker inspect -f '{{index .Config.Labels "ohk.gpu"}}' "$name")"
      if [ "$cur" = "$gpu" ] && [ "$(docker inspect -f '{{.State.Running}}' "$name")" = true ]; then echo "$name already up (gpu=$gpu)"; continue; fi
      docker rm -f "$name" >/dev/null
    fi
    local gargs=()
    [ "$gpu" = 1 ] && gargs=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all -e OHK_GPU=1)
    docker run -d --name "$name" --init --shm-size=2g -u "$UIDGID" --label "ohk.gpu=$gpu" --label ohk.sim=1 \
      -e HOME=/tmp -e PYTHONDONTWRITEBYTECODE=1 -e PYTHONUNBUFFERED=1 \
      -e ROS_DOMAIN_ID=$((40 + i)) -e GZ_PARTITION="$name" -e OHK_INSTANCE="$i" \
      -e TZ="${TZ:-$(timedatectl show -p Timezone --value 2>/dev/null || echo UTC)}" \
      -v "$VOL:/ws:ro" -v "$PROJECT:/ws/src/oomwoo_health_kit:ro" -v "$RES/$i:/results" \
      "${gargs[@]}" "$IMAGE" sleep infinity >/dev/null
    echo "$name up (domain $((40 + i)), gpu=$gpu)"
  done
}

running_run() { docker exec "$1" pgrep -f '[b]ench/fullstack/run.py' >/dev/null 2>&1; }

run() {
  local sel="${1:?usage: run <i|all> [--fg] [run.py args]}"; shift
  local fg=0
  [ "${1:-}" = "--fg" ] && { fg=1; shift; }
  local list; list="$(want "$sel")"
  [ -n "$list" ] || { echo "no containers" >&2; exit 4; }
  local kit; kit="$(git -C "$PROJECT" rev-parse HEAD 2>/dev/null || echo unknown)$(git -C "$PROJECT" diff --quiet HEAD -- 2>/dev/null || echo -dirty)"
  local agg; agg="$(docker exec "$(echo "$list" | head -1)" cat /ws/.agg_built 2>/dev/null || echo unknown)"
  local n
  for n in $list; do
    if running_run "$n"; then echo "$n: a run is already active (tools/sim.sh stop ${n#ohk-sim-})" >&2; exit 4; fi
    local args; args="$(printf '%q ' "$@")"
    local cmd="$WS_ENV && exec python3 -u /ws/src/oomwoo_health_kit/bench/fullstack/run.py $args"
    if [ "$fg" = 1 ]; then
      docker exec -u "$UIDGID" -e HOME=/tmp -e PYTHONDONTWRITEBYTECODE=1 -e PYTHONUNBUFFERED=1 -e OHK_KIT_COMMIT="$kit" -e OHK_AGG_COMMIT="$agg" \
        -w /ws "$n" bash -c "$cmd 2>&1 | tee -a /results/run.out"
    else
      docker exec -d -u "$UIDGID" -e HOME=/tmp -e PYTHONDONTWRITEBYTECODE=1 -e PYTHONUNBUFFERED=1 -e OHK_KIT_COMMIT="$kit" -e OHK_AGG_COMMIT="$agg" \
        -w /ws "$n" bash -c "$cmd >> /results/run.out 2>&1"
      echo "$n: started detached; log results/fullstack/${n#ohk-sim-}/run.log, data sessions.jsonl, stdout run.out"
    fi
  done
}

stop() {
  local n
  for n in $(want "${1:?usage: stop <i|all>}"); do
    docker exec "$n" bash -c '
      pids=$(pgrep -f "[b]ench/fullstack/run.py" || true)
      [ -n "$pids" ] || { echo "'"$n"': not running"; exit 0; }
      for p in $pids; do kill -TERM $p 2>/dev/null || true; done
      for i in $(seq 90); do pgrep -f "[b]ench/fullstack/run.py" >/dev/null || break; sleep 1; done
      for p in $(pgrep -f "[b]ench/fullstack/run.py" || true); do echo "SIGKILL run.py $p"; kill -KILL $p; done
      echo "'"$n"': stopped"'
  done
}

status() {
  local n any=0
  for n in $(names); do
    any=1
    local i="${n#ohk-sim-}" st act="idle"
    st="$(docker inspect -f '{{.State.Status}} gpu={{index .Config.Labels "ohk.gpu"}}' "$n" 2>/dev/null || echo '?') domain=$(docker exec "$n" printenv ROS_DOMAIN_ID 2>/dev/null || echo '?')"
    if [ "$(docker inspect -f '{{.State.Running}}' "$n")" = true ] && running_run "$n"; then act="RUN ACTIVE"; fi
    echo "== $n  $st  $act"
    [ -s "$RES/$i/progress.json" ] && echo "   progress: $(cat "$RES/$i/progress.json")"
    if [ -s "$RES/$i/sessions.jsonl" ]; then
      echo "   sessions: $(wc -l < "$RES/$i/sessions.jsonl"); last: $(tail -1 "$RES/$i/sessions.jsonl" | python3 -c '
import json,sys
d=json.loads(sys.stdin.read())
keys=["session","profile","valid","invalid_reason","goals_attempted","goals_succeeded","distance_m","soft_stops_n","hb_gaps_n","sim_s","wall_s","rtf","cpu_cores_container"]
print({k:d.get(k) for k in keys if k in d})')"
    fi
  done
  [ "$any" = 1 ] || echo "no ohk-sim-<i> containers"
  docker stats --no-stream --format '   {{.Name}} cpu={{.CPUPerc}} mem={{.MemUsage}}' $(names) 2>/dev/null || true
  echo "other containers (untouched):"; docker ps --format '{{.Names}}' | grep -v -E '^ohk-sim-[0-9]+$' | sed 's/^/   /' || true
}

down() {
  local n
  for n in $(names); do docker rm -f "$n" >/dev/null && echo "removed $n"; done
  docker rm -f ohk-sim-build >/dev/null 2>&1 || true
  if [ "${1:-}" = "--purge" ]; then docker volume rm "$VOL" >/dev/null 2>&1 && echo "removed volume $VOL" || true; fi
}

case "${1:-}" in
  build) build ;;
  up) shift; up "$@" ;;
  run) shift; run "$@" ;;
  stop) shift; stop "$@" ;;
  status) status ;;
  down) shift; down "$@" ;;
  exec) shift; n="ohk-sim-${1:?instance}"; shift; xrun "$n" "$*" ;;
  *) sed -n '2,3p' "$0" >&2; exit 2 ;;
esac
