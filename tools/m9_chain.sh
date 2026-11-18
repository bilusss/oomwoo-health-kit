#!/usr/bin/env bash
# M9: run several run.py invocations one after the other in one sim instance (each blocks until done). Usage:
#   OHK_RES_SUB=m9 OHK_OVERLAY=1 tools/m9_chain.sh <i> <file>      # one line per run = the arguments of `tools/sim.sh run <i> --fg ...`
# Stops at the first failing invocation. Output: results/fullstack/<OHK_RES_SUB>/<i>/run.out (written by sim.sh --fg) and chain.log (this script).
set -uo pipefail
i="${1:?instance}"; file="${2:?args file}"
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
log="$here/../results/fullstack/${OHK_RES_SUB:+$OHK_RES_SUB/}$i/chain.log"
mkdir -p "$(dirname "$log")"
while IFS= read -r line; do
  [ -z "$line" ] && continue
  case "$line" in \#*) continue ;; esac
  echo "$(date -Is) start: $line" >> "$log"
  # shellcheck disable=SC2086
  "$here/sim.sh" run "$i" --fg $line > /dev/null 2>&1
  rc=$?
  echo "$(date -Is) rc=$rc: $line" >> "$log"
  [ "$rc" = 0 ] || { echo "$(date -Is) chain stopped" >> "$log"; exit "$rc"; }
done < "$file"
echo "$(date -Is) chain done" >> "$log"
