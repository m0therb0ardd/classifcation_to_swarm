#!/usr/bin/env python3
import json
import time
import os
from cctl.api.network import Network
from cctl.api.bot_ctl import Coachbot

JSON_PATH = "swarm_config.json"
SEND_RATE = 10

# Mode to LED mapping (0-255 scale like MSI example)
MODE_COLORS = {
    "glitch": [255, 0, 255],      # Purple
    "move_left": [255, 0, 0],     # Red
    "move_right": [0, 255, 0],    # Green  
    "encircle": [255, 255, 0]     # Yellow
}

def main():
    net = Network().user
    last_mode = None
    
    print(f"MSI JSON Controller started. Watching: {JSON_PATH}")
    print("Available modes:", list(MODE_COLORS.keys()))

    while True:
        try:
            if not os.path.exists(JSON_PATH):
                time.sleep(1.0)
                continue
                
            with open(JSON_PATH, 'r') as f:
                data = json.load(f)
            
            mode = data.get("mode", "").strip().lower()
            
            if mode and mode != last_mode:
                print(f"Mode change: {last_mode} -> {mode}")
                last_mode = mode
                
                led_color = MODE_COLORS.get(mode, [255, 255, 255])  # default white
                
                # EXACT MSI format: "speed_x,speed_y;led_r,led_g,led_b"
                speed_data = '0.0,0.0'  # No movement
                led_data = ','.join(str(v) for v in led_color)
                encoded = f'{speed_data};{led_data}'.encode('ascii')
                
                print(f"Sending to bot 0: {encoded.decode()}")
                
                # EXACT MSI method: direct_signal to leader
                net.direct_signal('speed_led', Coachbot(0), encoded)
                
        except Exception as e:
            print(f"Error: {e}")
            
        time.sleep(1.0 / SEND_RATE)

if __name__ == "__main__":
    main()