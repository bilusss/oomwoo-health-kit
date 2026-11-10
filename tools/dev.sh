#!/usr/bin/env bash
# Dev container for the ROS parts of oomwoo-health-kit. Usage: tools/dev.sh {up|build|test|soak-start|soak-stop|shell|down|exec CMD...}
#
# OHK_AGG_REF selects the aggregator (xbattlax/health-monitor) commit under test; default b2b9796.
# Any commit sha, branch or tag works; PR heads are fetched too, e.g. for PR #2:
#   OHK_AGG_REF=2f39ac0405664ce07b7123e6d46d354747c6e14a tools/dev.sh build && ... test
# `build` rebuilds the aggregator from scratch when the ref changed since the last build; `test` refuses to run
# against a build from a different ref.
set -euo pipefail
NAME=ohk-dev
IMAGE=makerspet/oomwoo:jazzy-dev
AGG_URL=https://github.com/xbattlax/health-monitor.git
AGG_COMMIT="${OHK_AGG_REF:-b2b9796}"
PROJECT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UIDGID="$(id -u):$(id -g)"
# Run as the host user so nothing in the bind mount becomes root-owned.
EXEC=(docker exec -u "$UIDGID" -e HOME=/tmp -e PYTHONDONTWRITEBYTECODE=1 -e PYTEST_ADDOPTS="-p no:cacheprovider" -w /ws "$NAME")
SRC='source /opt/ros/jazzy/setup.bash'
WS='[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash'

SHM_BYTES=$((1024*1024*1024))  # Fast DDS leaks ~0.5 MB of /dev/shm per SIGKILLed participant; Docker's default is 64 MiB

up() {
  # An existing container created with a smaller /dev/shm is removed and recreated (the ohk-ws volume is kept).
  # Detection: docker inspect .HostConfig.ShmSize (bytes; the Docker default is 67108864).
  if docker container inspect "$NAME" >/dev/null 2>&1 \
     && [ "$(docker inspect -f '{{.HostConfig.ShmSize}}' "$NAME")" -lt "$SHM_BYTES" ]; then
    echo "recreating $NAME: /dev/shm is $(docker inspect -f '{{.HostConfig.ShmSize}}' "$NAME") bytes, need $SHM_BYTES" >&2
    docker rm -f "$NAME" >/dev/null
  fi
  if ! docker container inspect "$NAME" >/dev/null 2>&1; then
    docker volume create ohk-ws >/dev/null
    docker run -d --name "$NAME" --init --shm-size=1g -v ohk-ws:/ws \
      -v "$PROJECT:/ws/src/oomwoo_health_kit" \
      -e PYTHONDONTWRITEBYTECODE=1 "$IMAGE" sleep infinity >/dev/null
  elif [ "$(docker inspect -f '{{.State.Running}}' "$NAME")" != true ]; then
    docker start "$NAME" >/dev/null
  fi
  docker exec -u root "$NAME" bash -c 'chown '"$UIDGID"' /ws /ws/src'
  "${EXEC[@]}" bash -c "[ -d /ws/src/health-monitor/.git ] || git clone -q $AGG_URL /ws/src/health-monitor
    cd /ws/src/health-monitor
    git rev-parse -q --verify '$AGG_COMMIT^{commit}' >/dev/null 2>&1 \
      || git fetch -q origin '+refs/heads/*:refs/remotes/origin/*' '+refs/pull/*/head:refs/remotes/origin/pr/*'
    git checkout -q '$AGG_COMMIT'"
}

agg_sha() { "${EXEC[@]}" git -C /ws/src/health-monitor rev-parse HEAD; }
built_sha() { "${EXEC[@]}" bash -c 'cat /ws/.agg_built 2>/dev/null || true'; }

build() {
  up
  local want have
  want="$(agg_sha)"; have="$(built_sha)"
  echo "aggregator ref: $AGG_COMMIT ($want)"
  if [ "$want" != "$have" ]; then
    "${EXEC[@]}" bash -c 'rm -rf /ws/build/oomwoo_health_monitor /ws/install/oomwoo_health_monitor'
  fi
  "${EXEC[@]}" bash -c "$SRC && colcon build --parallel-workers 6 --event-handlers console_direct+ $*"
  "${EXEC[@]}" bash -c "echo $want > /ws/.agg_built"
}

run_tests() {
  up
  local want have
  want="$(agg_sha)"; have="$(built_sha)"
  echo "aggregator ref: $AGG_COMMIT ($want)"
  if [ "$want" != "$have" ]; then
    echo "installed build is from aggregator ${have:-<none>}; run: OHK_AGG_REF=$AGG_COMMIT tools/dev.sh build" >&2
    exit 3
  fi
  "${EXEC[@]}" bash -c "$SRC && $WS && export OHK_RESULTS=\${OHK_RESULTS:-/tmp/ohk_results.json} && colcon test --packages-select oomwoo_health_kit --parallel-workers 1 --event-handlers console_direct+ $*; rc=\$?; colcon test-result --verbose; exit \$rc"
}

case "${1:-}" in
  up) up ;;
  build) shift; build "$@" ;;
  test) shift; run_tests "$@" ;;
  soak-start) shift  # detached long run; survives this shell. Args go to bench/soak.py, e.g. --quiet-window A/B
    up
    if "${EXEC[@]}" pgrep -f '[b]ench/soak.py' >/dev/null; then echo "soak already running (use soak-stop first)" >&2; exit 4; fi
    kit="$(git -C "$PROJECT" rev-parse HEAD)$(git -C "$PROJECT" diff --quiet HEAD -- 2>/dev/null || echo -dirty)"
    docker exec -d -u "$UIDGID" -e HOME=/tmp -e PYTHONDONTWRITEBYTECODE=1 -e PYTHONUNBUFFERED=1 \
      -e TZ="${TZ:-$(timedatectl show -p Timezone --value 2>/dev/null || echo UTC)}" \
      -e OHK_KIT_COMMIT="$kit" -e OHK_AGG_COMMIT="$(agg_sha)" -w /ws "$NAME" \
      bash -c "$SRC && $WS && exec python3 /ws/src/oomwoo_health_kit/bench/soak.py $* >> /ws/src/oomwoo_health_kit/results/soak.out 2>&1"
    echo "started (kit $kit); log: results/soak.log, data: results/soak.jsonl, stdout: results/soak.out" ;;
  soak-stop)  # SIGTERM, wait up to 60 s, then SIGKILL; children die with the parent (PDEATHSIG); report leftovers
    "${EXEC[@]}" bash -c '
      pids=$(pgrep -f "[b]ench/soak.py" || true)
      [ -n "$pids" ] || echo "soak not running"
      for p in $pids; do kill -TERM $p 2>/dev/null || true; done
      for i in $(seq 60); do pgrep -f "[b]ench/soak.py" >/dev/null || break; sleep 1; done
      for p in $(pgrep -f "[b]ench/soak.py" || true); do echo "SIGKILL soak $p"; kill -KILL $p; done
      sleep 1
      left=$(pgrep -a -f "[o]omwoo_health_monitor/|[o]omwoo_health_kit/|while True: [p]ass|[o]hk_soak_" || true)
      if [ -n "$left" ]; then echo "LEFTOVER processes (killing):"; echo "$left"; pkill -KILL -f "[o]omwoo_health_monitor/|[o]omwoo_health_kit/|while True: [p]ass|[o]hk_soak_" || true; else echo "no leftover processes"; fi' ;;
  shell) up; docker exec -it -u "$UIDGID" -e HOME=/tmp -e PYTHONDONTWRITEBYTECODE=1 -w /ws "$NAME" bash --rcfile <(echo "$SRC; $WS") ;;
  exec) up; "${EXEC[@]}" bash -c "$SRC && $WS && ${*:2}" ;;
  down) docker rm -f "$NAME" >/dev/null 2>&1 || true; docker volume rm ohk-ws >/dev/null 2>&1 || true ;;
  *) echo "usage: $0 {up|build|test|soak-start [soak.py args]|soak-stop|shell|exec CMD|down}" >&2; exit 2 ;;
esac
