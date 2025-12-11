#!/usr/bin/env python3

import os
import sys
import yaml
import random
import time
import asyncio
import argparse
import termios
import tty
from pathlib import Path

# Ensure we can import from src/codebotler
# Assuming the script is run from the workspace root or src/codebotler/scripts
# We want to add 'src/codebotler' to path so we can import cli
# Or better, add 'src' so we can do 'from codebotler import cli'

CURRENT_DIR = Path(__file__).resolve().parent
# Add src/codebotler to sys.path
CODEBOTLER_DIR = CURRENT_DIR.parent
SRC_DIR = CODEBOTLER_DIR.parent
sys.path.insert(0, str(CODEBOTLER_DIR))

print(f"DEBUG: CURRENT_DIR={CURRENT_DIR}")
print(f"DEBUG: CODEBOTLER_DIR={CODEBOTLER_DIR}")
print(f"DEBUG: sys.path[0]={sys.path[0]}")
print(f"DEBUG: Files in CODEBOTLER_DIR: {list(CODEBOTLER_DIR.iterdir())}")

try:
    import cli
except ImportError as e:
    print(f"Error: Could not import 'cli': {e}")
    sys.exit(1)

COMMAND_TEMPLATES = {
    "move": "Go to {{location}}.",
    "observe": "If there is an {{object}} in the room, say yes",
    #"say": "Say {{phrase}}"
    #"pick": "Grab the {{object}}.",
    "pick": "If there is an {{object}}, grab it. If not say that the {{object}} does not exist"
    #"place": "Place the object in the bin"
}

OBJECTS = ["can", "cup", "chair", "bottle", "book", "laptop", "phone", "notebook", "pen", "backpack", "keyboard", "mouse",  "table", "lamp"]
PICK_OBJECTS = ["soda can", "cup", "plastic bottle"]
PHRASES = [
    "Hello",
    "How are you?",
    "I am a robot.",
    "I like to help humans.",
    "I am testing Codebotler.",
    "I will be moving around.",
    "I will be looking for objects.",
]

def load_locations():
    data_path = SRC_DIR / "codebotler_amrl_impl" / "data.yaml"
    if not data_path.exists():
        print(f"Error: Could not find data.yaml at {data_path}")
        return []
    
    with open(data_path, "r") as f:
        data = yaml.safe_load(f)
    
    map_name = data.get("MAP", "ahg2apt")
    locations_dict = data.get("LOCATIONS", {}).get(map_name, {})
    return list(locations_dict.keys())

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def wait_for_user_input():
    print("Press Enter or Space to continue, Esc to quit...")
    while True:
        key = get_key()
        if key == '\x1b': # Escape key
            return False
        if key == ' ' or key == '\r' or key == '\n':
            return True
        # Ignore other keys

def main():
    parser = argparse.ArgumentParser(description="Integration test for Codebotler")
    parser.add_argument("--single-step", action="store_true", help="Enable single-step mode (wait for user input before each command)")
    args = parser.parse_args()

    locations = load_locations()
    if not locations:
        print("No locations found. Exiting.")
        return

    print(f"Loaded {len(locations)} locations: {locations}")
    
    last_action = None
    
    # Default server config
    host = "10.1.0.13"
    port = 8190
    uri = f"ws://{host}:{port}"

    try:
        while True:
            if args.single_step:
                if not wait_for_user_input():
                    print("\nQuitting...")
                    break

            command_type = ""
            command_text = ""
            
            if last_action == "pick" and False:
                command_type = "place"
                command_text = COMMAND_TEMPLATES["place"]
            else:
                # Sample action type (move or pick)
                command_type = random.choice(list(COMMAND_TEMPLATES.keys()))
                
                if command_type == "move":
                    loc = random.choice(locations)
                    command_text = COMMAND_TEMPLATES["move"].replace("{{location}}", loc)
                elif command_type == "say":
                    phrase = random.choice(PHRASES)
                    command_text = COMMAND_TEMPLATES[command_type].replace("{{phrase}}", phrase)
                elif command_type == "observe":
                    obj = random.choice(OBJECTS)
                    command_text = COMMAND_TEMPLATES[command_type].replace("{{object}}", obj)
                elif command_type == "pick":
                    obj = random.choice(PICK_OBJECTS)
                    command_text = COMMAND_TEMPLATES[command_type].replace("{{object}}", obj)
            
            print(f"\n--- Spending command: {command_text} ---")
            
            # Send to codebotler via cli
            # We use the programmatic interface we added to cli.py
            response = asyncio.run(cli.send_request(uri, command_text, execute=True, verbose=True))
            
            if "error" in response:
                print("Failed to send command. Is the server running?")
                # Wait a bit before retrying to strictly avoid spamming failed connect requests
                time.sleep(5)
            else:
                 # Update state
                last_action = command_type
                
                # Wait a bit before next command
                time.sleep(5)

    except KeyboardInterrupt:
        print("\nStopping integration test.")

if __name__ == "__main__":
    main()
