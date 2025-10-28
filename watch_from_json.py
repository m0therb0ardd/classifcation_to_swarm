#!/usr/bin/python3.8
import json, subprocess, time, os, sys
from datetime import datetime

CONFIG_JSON = os.path.join(os.path.dirname(__file__), "swarm_config.json")

MODE_TO_FILE = {
    "float": "usr_code_filler.py",
    "glide": "usr_code_filler.py",
    "glitch": "usr_code_glitch.py",
    "directional_left": "usr_code_move_left.py",
    "directional_right": "usr_code_move_right.py",
    "punch": "usr_code_filler.py",
    "slash": "usr_code_filler.py",
    "encircling": "usr_code_encircling.py",
}

ROBOTS = ["34", "35", "36"]

def sh(*args, timeout=None):
    """Helper to run CLI commands and print timestamped logs."""
    print(f"[{datetime.now().strftime('%H:%M:%S')}] +", " ".join(args))
    subprocess.run(args, check=True, timeout=timeout)

def read_json():
    """Safely read JSON, returning dict or empty."""
    try:
        with open(CONFIG_JSON, "r") as f:
            return json.load(f)
    except Exception:
        return {}

def apply_mode(mode):
    script = MODE_TO_FILE.get(mode)
    if not script or not os.path.isfile(script):
        print(f"[WARN] No script for mode '{mode}' or file missing.")
        return

    abs_script = os.path.abspath(script)
    print(f"[{datetime.now().strftime('%H:%M:%S')}] Applying mode '{mode}' using {abs_script}")

    # power on
    sh("cctl", "on", *ROBOTS)

    # update code
    sh("cctl", "update", abs_script, timeout=60)

    # start bots
    sh("cctl", "start", *ROBOTS, timeout=30)

def main():
    print(f"[watch] Watching {CONFIG_JSON} for mode changes...")
    last_mode, last_ts = None, None

    while True:
        cfg = read_json()
        mode, ts = cfg.get("mode"), cfg.get("timestamp")

        if mode and (mode != last_mode or ts != last_ts):
            print(f"[{datetime.now().strftime('%H:%M:%S')}] Change detected: {last_mode} → {mode}")
            apply_mode(mode)
            last_mode, last_ts = mode, ts

        time.sleep(1)  # poll once per second

if __name__ == "__main__":
    main()
