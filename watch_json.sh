#!/usr/bin/env bash
set -euo pipefail

JSON="${1:-swarm_config.json}"
ROBOTS="${ROBOTS:-34 35 36}"
SLEEP_SEC="${SLEEP_SEC:-1}"
UPDATE_TIMEOUT="${UPDATE_TIMEOUT:-120s}"

declare -A MODE_TO_FILE=(
  ["float"]="usr_code_filler.py"
  ["glide"]="usr_code_filler.py"
  ["glitch"]="usr_code_glitch.py"
  ["directional_left"]="usr_code_move_left.py"
  ["directional_right"]="usr_code_move_right.py"
  ["punch"]="usr_code_filler.py"
  ["slash"]="usr_code_filler.py"
  ["encircling"]="usr_code_encircling.py"
)

ts(){ date +"%Y-%m-%d %H:%M:%S"; }

read_field() {
  local field="$1"
  python3 - "$JSON" <<PY 2>/dev/null || true
import json,sys
try:
    with open(sys.argv[1]) as f: d=json.load(f)
    v=d.get("$field","")
    print(v if isinstance(v,str) else ("" if v is None else v))
except: pass
PY
}

apply_mode() {
  local mode="$1"
  local script_rel="${MODE_TO_FILE[$mode]:-}"

  if [[ -z "$script_rel" ]]; then
    echo "[$(ts)] [watch] No script mapped for mode '$mode' — skipping."
    return
  fi
  if [[ ! -f "$script_rel" ]]; then
    echo "[$(ts)] [watch] ERROR: mapped file missing: $script_rel"
    return
  fi
  local script_abs
  script_abs="$(readlink -f "$script_rel")"

  echo "[$(ts)] [watch] APPLY: mode '$mode' -> $script_rel"
  echo "[$(ts)] [watch] Powering ON robots: $ROBOTS"
  cctl on $ROBOTS

  echo "[$(ts)] [watch] Updating ALL ON bots: $script_abs  (timeout $UPDATE_TIMEOUT)"
  if timeout "$UPDATE_TIMEOUT" cctl update "$script_abs"; then
    echo "[$(ts)] [watch] Update OK."
  else
    echo "[$(ts)] [watch] Update timed out/failed; continuing."
  fi

  echo "[$(ts)] [watch] Starting user code on ALL ON bots"
  cctl start
}

echo "[$(ts)] [watch] Watching $JSON (ROBOTS=$ROBOTS, every ${SLEEP_SEC}s)"
last_mode="__none__"
last_ts="__none__"

while true; do
  if [[ -f "$JSON" ]]; then
    mode="$(read_field mode)"
    tsval="$(read_field timestamp)"
    if [[ -n "$mode" && ( "$mode" != "$last_mode" || "$tsval" != "$last_ts" ) ]]; then
      echo "[$(ts)] [watch] Change: mode '$last_mode'→'$mode', ts '$last_ts'→'$tsval'"
      apply_mode "$mode"
      last_mode="$mode"
      last_ts="$tsval"
    fi
  fi
  sleep "$SLEEP_SEC"
done
