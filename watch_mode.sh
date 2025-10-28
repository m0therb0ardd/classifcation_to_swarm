#!/usr/bin/env bash
set -euo pipefail

# ========== CONFIG YOU CAN EDIT ==========
CONFIG_JSON="${1:-swarm_config.json}"
ROBOTS="${ROBOTS:-34 35 36}"
SLEEP_SEC="${SLEEP_SEC:-1}"

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

CCTL_ARGS="${CCTL_ARGS:-}"
# ========================================

need() { command -v "$1" >/dev/null 2>&1 || { echo "Missing dependency: $1"; exit 1; }; }

# ❌ remove jq dependency
# need jq
# ✅ require python3 instead
need python3
# (optional) require inotifywait if you don't want polling:
# need inotifywait

# comment out to force DRYRUN even if cctl exists
# need cctl

# DRYRUN shim (keeps exact commands visible)
if ! command -v cctl >/dev/null 2>&1; then
  echo "[watch] cctl not found; DRY-RUN mode enabled."
  cctl() { echo "DRYRUN cctl $*"; }
fi

have_inotify=0
if command -v inotifywait >/dev/null 2>&1; then
  have_inotify=1
fi

ts() { date +"%Y-%m-%d %H:%M:%S"; }

# ---- JSON readers using python (no jq needed) ----
read_mode() {
  python3 - "$CONFIG_JSON" <<'PY' 2>/dev/null || true
import json, sys
try:
    with open(sys.argv[1], 'r') as f:
        d = json.load(f)
    print(d.get('mode',''))
except Exception:
    pass
PY
}

read_timestamp() {
  python3 - "$CONFIG_JSON" <<'PY' 2>/dev/null || true
import json, sys
try:
    with open(sys.argv[1], 'r') as f:
        d = json.load(f)
    v = d.get('timestamp','')
    print(v if isinstance(v, str) else ('' if v is None else v))
except Exception:
    pass
PY
}

echo "[$(ts)] [watch] Using config: $CONFIG_JSON"
if [[ ! -f "$CONFIG_JSON" ]]; then
  echo "[$(ts)] [watch] WARNING: $CONFIG_JSON not found yet; waiting for it to appear…"
fi

echo "[$(ts)] [watch] Selecting robots: $ROBOTS"
cctl on $ROBOTS

last_mode="__none__"
last_seen_ts="__none__"

apply_mode() {
  local mode="$1"
  local script="${MODE_TO_FILE[$mode]:-}"

  if [[ -z "$script" ]]; then
    echo "[$(ts)] [watch] No script mapped for mode '$mode' — ignoring."
    return 0
  fi
  if [[ ! -f "$script" ]]; then
    echo "[$(ts)] [watch] ERROR: mapped file '$script' not found."
    return 0
  fi

  echo "[$(ts)] [watch] Mode -> '$mode' | Script -> $script"

  echo "[$(ts)] [watch] cctl pause"
  cctl $CCTL_ARGS pause || echo "[$(ts)] [watch] (pause skipped or already paused)"

  echo "[$(ts)] [watch] cctl update $script"
  cctl $CCTL_ARGS update "$script"

  echo "[$(ts)] [watch] cctl start"
  cctl $CCTL_ARGS start
}

react_if_changed() {
  [[ ! -f "$CONFIG_JSON" ]] && return 0

  local mode ts_val
  mode="$(read_mode)"
  ts_val="$(read_timestamp)"
  [[ -z "$mode" ]] && return 0

  if [[ "$mode" != "$last_mode" || "$ts_val" != "$last_seen_ts" ]]; then
    echo "[$(ts)] [watch] Change detected: mode '$last_mode'→'$mode', ts '$last_seen_ts'→'$ts_val'"
    apply_mode "$mode"
    last_mode="$mode"
    last_seen_ts="$ts_val"
  fi
}

# Initial apply if present
react_if_changed

if [[ $have_inotify -eq 1 ]]; then
  echo "[$(ts)] [watch] Watching with inotifywait…"
  while inotifywait -q -e close_write -e move -e attrib "$(dirname "$CONFIG_JSON")" >/dev/null; do
    react_if_changed
  done
else
  echo "[$(ts)] [watch] inotifywait not found; polling every $SLEEP_SEC s…"
  while true; do
    sleep "$SLEEP_SEC"
    react_if_changed
  done
fi
