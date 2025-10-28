#!/usr/bin/env bash
set -euo pipefail

# ====== config you can tweak ======
JSON="${1:-swarm_config.json}"         # path to your JSON
ROBOTS="${ROBOTS:-34 35 36}"           # env override: ROBOTS="3 4 5" ./apply_from_json.sh
UPDATE_TIMEOUT="${UPDATE_TIMEOUT:-120s}"
# map mode -> file
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
# ==================================

ts(){ date +"%Y-%m-%d %H:%M:%S"; }

if [[ ! -f "$JSON" ]]; then
  echo "[$(ts)] [apply] ERROR: JSON not found: $JSON"
  exit 1
fi

# Read "mode" with Python (no jq needed)
read_mode() {
  python3 - "$JSON" <<'PY' 2>/dev/null || true
import json,sys
try:
    with open(sys.argv[1]) as f: d=json.load(f)
    print(d.get("mode",""))
except: pass
PY
}

MODE="$(read_mode)"
if [[ -z "$MODE" ]]; then
  echo "[$(ts)] [apply] ERROR: .mode missing/empty in $JSON"
  exit 1
fi

SCRIPT_REL="${MODE_TO_FILE[$MODE]:-}"
if [[ -z "$SCRIPT_REL" ]]; then
  echo "[$(ts)] [apply] ERROR: no script mapped for mode '$MODE'"
  exit 1
fi
if [[ ! -f "$SCRIPT_REL" ]]; then
  echo "[$(ts)] [apply] ERROR: mapped file missing: $SCRIPT_REL"
  exit 1
fi
SCRIPT_ABS="$(readlink -f "$SCRIPT_REL")"

echo "[$(ts)] [apply] Mode: '$MODE'  ->  Script: $SCRIPT_REL"
echo "[$(ts)] [apply] Powering ON robots: $ROBOTS"
cctl on $ROBOTS

echo "[$(ts)] [apply] Updating user code to ALL ON bots: $SCRIPT_ABS"
timeout "$UPDATE_TIMEOUT" cctl update "$SCRIPT_ABS" || {
  echo "[$(ts)] [apply] WARNING: update timed out/failed; continuing"
}

echo "[$(ts)] [apply] Starting user code on ALL ON bots"
cctl start

echo "[$(ts)] [apply] Done."
