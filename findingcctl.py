import json
import time
import os

# Try multiple import strategies
try:
    from cctl.api.network import Network
    from cctl.api.bot_ctl import Coachbot
    print("Using cctl.api imports")
except ImportError:
    try:
        from cctl import Network, Coachbot
        print("Using direct cctl imports")
    except ImportError:
        try:
            import cctl
            Network = cctl.Network
            Coachbot = cctl.Coachbot
            print("Using cctl module attributes")
        except ImportError:
            print("ERROR: Could not import cctl. Please check installation.")
            exit(1)

JSON_PATH = "swarm_config.json"
SEND_HZ = 2

def main():
    net = Network().user
    last_mode = None
    
    print("Controller watching: " + JSON_PATH)
    print("Available modes: " + str(["idle", "float", "glitch", "encircling", "stillness"]))

    while True:
        try:
            if not os.path.exists(JSON_PATH):
                print("JSON file not found: " + JSON_PATH)
                time.sleep(1.0)
                continue
                
            with open(JSON_PATH, "r") as f:
                data = json.load(f)
            mode = (data.get("mode") or "").strip().lower()
            
            if mode and mode != last_mode:
                print("Mode change: " + str(last_mode) + " -> " + mode)
                last_mode = mode
                
                net.direct_signal("mode_host", Coachbot(10), mode.encode("utf-8"))
                print("Sent mode '" + mode + "' to leader bot 10")
                
        except Exception as e:
            print("Controller error: " + str(e))
            
        time.sleep(1.0 / SEND_HZ)

if __name__ == "__main__":
    main()