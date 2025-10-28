#!/usr/bin/env bash
set -euo pipefail

# ================= CONFIG =================
ROBOTS="34 35 36"
SCRIPT_PATH="$(pwd)/usr_code_glitch.py"   # update this if needed
UPDATE_TIMEOUT="120s"                     # how long to wait for update
# ==========================================

ts() { date +"%Y-%m-%d %H:%M:%S"; }

echo "[$(ts)] [run] Turning on robots: $ROBOTS"
cctl on $ROBOTS
echo "[$(ts)] [run] Power ON complete."

echo "[$(ts)] [run] Updating user code: $SCRIPT_PATH"
timeout "$UPDATE_TIMEOUT" cctl update "$SCRIPT_PATH" || {
  echo "[$(ts)] [run] WARNING: update timed out or failed."
}
echo "[$(ts)] [run] Update complete."

echo "[$(ts)] [run] Starting user code on all ON bots..."
cctl start
echo "[$(ts)] [run] Start command sent."

# ----- wait for 20 seconds -----
PAUSE_DELAY=20
echo "[$(ts)] [run] Waiting ${PAUSE_DELAY}s before pausing..."
sleep "$PAUSE_DELAY"

echo "[$(ts)] [run] Pausing all ON bots..."
cctl pause
echo "[$(ts)] [run] Pause complete."

echo "[$(ts)] [run] Done."
