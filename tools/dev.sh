#!/usr/bin/env bash
# Dev container for the ROS parts of oomwoo-health-kit. Usage: tools/dev.sh {up|build|test|shell|down|exec CMD...}
set -euo pipefail
NAME=ohk-dev
IMAGE=makerspet/oomwoo:jazzy-dev
AGG_URL=https://github.com/xbattlax/health-monitor.git
AGG_COMMIT=b2b9796
PROJECT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UIDGID="$(id -u):$(id -g)"
# Run as the host user so nothing in the bind mount becomes root-owned.
EXEC=(docker exec -u "$UIDGID" -e HOME=/tmp -e PYTHONDONTWRITEBYTECODE=1 -e PYTEST_ADDOPTS="-p no:cacheprovider" -w /ws "$NAME")
SRC='source /opt/ros/jazzy/setup.bash'
WS='[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash'

up() {
  if ! docker container inspect "$NAME" >/dev/null 2>&1; then
    docker volume create ohk-ws >/dev/null
    docker run -d --name "$NAME" --init -v ohk-ws:/ws \
      -v "$PROJECT:/ws/src/oomwoo_health_kit" \
      -e PYTHONDONTWRITEBYTECODE=1 "$IMAGE" sleep infinity >/dev/null
  elif [ "$(docker inspect -f '{{.State.Running}}' "$NAME")" != true ]; then
    docker start "$NAME" >/dev/null
  fi
  docker exec -u root "$NAME" bash -c 'chown '"$UIDGID"' /ws /ws/src'
  "${EXEC[@]}" bash -c "[ -d /ws/src/health-monitor/.git ] || git clone -q $AGG_URL /ws/src/health-monitor
    git -C /ws/src/health-monitor checkout -q $AGG_COMMIT"
}

case "${1:-}" in
  up) up ;;
  build) up; "${EXEC[@]}" bash -c "$SRC && colcon build --parallel-workers 6 --event-handlers console_direct+ ${*:2}" ;;
  test) up; "${EXEC[@]}" bash -c "$SRC && $WS && export OHK_RESULTS=\${OHK_RESULTS:-/tmp/ohk_results.json} && colcon test --packages-select oomwoo_health_kit --parallel-workers 1 --event-handlers console_direct+ ${*:2}; rc=\$?; colcon test-result --verbose; exit \$rc" ;;
  shell) up; docker exec -it -u "$UIDGID" -e HOME=/tmp -e PYTHONDONTWRITEBYTECODE=1 -w /ws "$NAME" bash --rcfile <(echo "$SRC; $WS") ;;
  exec) up; "${EXEC[@]}" bash -c "$SRC && $WS && ${*:2}" ;;
  down) docker rm -f "$NAME" >/dev/null 2>&1 || true; docker volume rm ohk-ws >/dev/null 2>&1 || true ;;
  *) echo "usage: $0 {up|build|test|shell|exec CMD|down}" >&2; exit 2 ;;
esac
