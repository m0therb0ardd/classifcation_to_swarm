# #!/usr/bin/env python3
# import asyncio, json, os, time
# from argparse import Namespace
# from cctl import cli
# from cctl.conf import Configuration

# # Which robots to control (edit as needed)
# ROBOTS = [34, 35, 36]

# # 1. Read the mode from swarm_config.json
# def read_mode():
#     try:
#         with open("swarm_config.json") as f:
#             data = json.load(f)
#         return data.get("mode", ""), data.get("timestamp", "")
#     except Exception as e:
#         print("Error reading json:", e)
#         return "", ""

# # 2. Map mode to file
# MODE_TO_FILE = {
#     "float": "usr_code_filler.py",
#     "glide": "usr_code_filler.py",
#     "glitch": "usr_code_glitch.py",
#     "directional_left": "usr_code_move_left.py",
#     "directional_right": "usr_code_move_right.py",
#     "punch": "usr_code_filler.py",
#     "slash": "usr_code_filler.py",
#     "encircling": "usr_code_encircling.py",
# }

# async def main():
#     mode, ts = read_mode()
#     if not mode:
#         print("No mode found.")
#         return
#     script = MODE_TO_FILE.get(mode)
#     if not script or not os.path.exists(script):
#         print("Script not found for mode:", mode)
#         return
#     print(f"[INFO] Mode: {mode}  Script: {script}")

#     # 3. Turn robots ON
#     args = Namespace(id=[str(i) for i in ROBOTS], force=True)
#     print("[INFO] Turning on robots", ROBOTS)
#     await cli.commands.on_handle(args, Configuration())

#     # 4. Upload user code (update runs on all ON bots)
#     args = Namespace(usr_path=[os.path.abspath(script)], os_update=False)
#     print("[INFO] Updating code on ON bots")
#     await cli.commands.update_handler(args, Configuration())

#     # 5. Start the selected robots
#     args = Namespace(id=[str(i) for i in ROBOTS])
#     print("[INFO] Starting robots")
#     await cli.commands.start_handle(args, Configuration())

#     # 6. Optional pause after delay
#     PAUSE_AFTER_SEC = 20
#     print(f"[INFO] Sleeping {PAUSE_AFTER_SEC}s before pausing…")
#     await asyncio.sleep(PAUSE_AFTER_SEC)
#     print("[INFO] Pausing robots")
#     await cli.commands.pause_handle(args, Configuration())

# if __name__ == "__main__":
#     asyncio.run(main())


#!/usr/bin/env python3
import asyncio, json, os, time
from cctl import cli
from cctl.conf import Configuration

ROBOTS = [34, 35, 36]
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
JSON_PATH = "swarm_config.json"
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

    print("[INFO] Selecting robots once:", ROBOTS)
    await exec_cctl(conf, *ROBOTS)  # same as: cctl 34 35 36

    last_mode, last_ts = "", ""
    while True:
        mode, ts = read_mode_ts()
        if mode and (mode != last_mode or ts != last_ts):
            script = MODE_TO_FILE.get(mode)
            if not script or not os.path.exists(script):
                print(f"[WARN] No script for mode '{mode}' or file missing.")
            else:
                abspath = os.path.abspath(script)
                print(f"[INFO] Mode → {mode} | updating {abspath}")
                await exec_cctl(conf, "user-code", "code", "update", "--file", abspath)
                # optional LED cue here with your CLI command if available
                print("[INFO] Starting user-code")
                # stop (ignore failure) then start for a clean swap
                try:
                    await exec_cctl(conf, "user-code", "running", "delete")
                except Exception:
                    pass
                await exec_cctl(conf, "user-code", "running", "create")
            last_mode, last_ts = mode, ts
        await asyncio.sleep(POLL)

if __name__ == "__main__":
    asyncio.run(main())

