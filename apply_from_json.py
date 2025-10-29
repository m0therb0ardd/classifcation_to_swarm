#!/usr/bin/env python3.8
import asyncio, json, os
from cctl import cli
from cctl.conf import Configuration

ROBOTS = [4, 5]
MODE_TO_FILE = {
    "float":            "usr_code_filler.py",
    "glide":            "usr_code_filler.py",
    "glitch":           "usr_code_glitch.py",
    "directional_left": "usr_code_move_left.py",
    "directional_right":"usr_code_move_right.py",
    "punch":            "usr_code_filler.py",
    "slash":            "usr_code_filler.py",
    "encircling":       "usr_code_encircling.py",
}

HERE = os.path.abspath(os.path.dirname(__file__))
JSON_PATH = os.path.join(HERE, "swarm_config.json")
POLL = 1.0  # seconds

def read_mode_ts():
    try:
        with open(JSON_PATH) as f:
            d = json.load(f)
        return (d.get("mode") or "").strip().lower(), d.get("timestamp", "")
    except Exception:
        return "", ""

async def exec_cctl(conf, *argv):
    parser = cli.create_parser()
    args = parser.parse_args(list(map(str, argv)))
    return await cli.exec_command(args, conf)

async def main():
    conf = Configuration()

    # 1) Power on / select robots once
    print("[INFO] Powering on / selecting robots:", ROBOTS)
    try:
        await exec_cctl(conf, "on", *ROBOTS)  # cctl on 4 5
    except Exception as e:
        print("[WARN] 'cctl on' failed:", e)

    last_mode, last_ts = "", ""
    while True:
        mode, ts = read_mode_ts()
        if mode and (mode != last_mode or ts != last_ts):
            script_rel = MODE_TO_FILE.get(mode)
            if not script_rel:
                print(f"[WARN] No script mapped for mode '{mode}'.")
            else:
                script_abs = os.path.abspath(os.path.join(HERE, script_rel))
                if not os.path.exists(script_abs):
                    print(f"[WARN] Script file missing: {script_abs}")
                else:
                    print(f"[INFO] Mode → {mode} | PAUSE → UPDATE → START")
                    # 2) Pause the running user code on selected robots
                    try:
                        await exec_cctl(conf, "pause", *ROBOTS)       # cctl pause 4 5
                    except Exception as e:
                        print("[WARN] 'cctl pause' failed (continuing):", e)

                    # (tiny debounce helps some BLE stacks)
                    await asyncio.sleep(0.2)

                    # 3) Push new user code
                    try:
                        await exec_cctl(conf, "update", script_abs)   # cctl update /abs/path.py
                    except Exception as e:
                        print("[ERROR] 'cctl update' failed:", e)
                        # don't advance last_* so we retry on next poll
                        await asyncio.sleep(POLL)
                        continue

                    # Another tiny delay before restart
                    await asyncio.sleep(0.2)

                    # 4) Start on selected robots
                    try:
                        await exec_cctl(conf, "start", *ROBOTS)       # cctl start 4 5
                        print("[INFO] Started:", ROBOTS)
                    except Exception as e:
                        print("[WARN] 'cctl start' failed:", e)

            last_mode, last_ts = mode, ts
        await asyncio.sleep(POLL)

if __name__ == "__main__":
    asyncio.run(main())
