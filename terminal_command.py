python3 - <<'PY'
import json, time
from cctl.api.network import Network
net = Network().user
last = None
while True:
    try:
        with open("swarm_config.json","r") as f: mode = json.load(f).get("mode","")
    except Exception: mode = ""
    if mode and mode != last:
        net.signal("mode", mode.encode("utf-8"))
        print("sent from json:", mode)
        last = mode
    time.sleep(0.25)
PY



python3 - <<'PY'
from cctl.api.network import Network
net = Network().user
net.signal('mode', b'glitch')
print("sent: glitch")
PY
