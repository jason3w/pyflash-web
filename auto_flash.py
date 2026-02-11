import subprocess
import time
import os
import sys
import signal
import json
import threading
import http.server
import socketserver
import webbrowser

# Configuration
def resource_path(relative_path):
    """ Get absolute path to resource, works for dev and for PyInstaller """
    try:
        base_path = sys._MEIPASS
    except Exception:
        base_path = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(base_path, relative_path)

# Paths for bundled resources
PACK_PATH = resource_path("Puya.PY32F0xx_DFP.1.2.8.pack")
# HEX_PATH and other config will be handled dynamically or from current dir
TARGET = "py32f002bx5"
STATUS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "status.json")

# Global state for statuses
probe_status = {}
status_lock = threading.Lock()


def update_status(uid, status):
    """Writes the current status of a specific probe to a JSON file."""
    with status_lock:
        if uid not in probe_status:
            probe_status[uid] = {"status": "READY", "last_seen": time.time()}
        
        probe_status[uid]["status"] = status
        probe_status[uid]["last_seen"] = time.time()
        
        try:
            with open(STATUS_FILE, "w") as f:
                json.dump(probe_status, f)
        except Exception as e:
            print(f"Error updating status: {e}")

def remove_probe_status(uid):
    """Removes a probe from the status file when it's disconnected."""
    with status_lock:
        if uid in probe_status:
            del probe_status[uid]
            try:
                with open(STATUS_FILE, "w") as f:
                    json.dump(probe_status, f)
            except Exception as e:
                print(f"Error updating status: {e}")


def run_command(command, check=False, stream_output=False):
    """Runs a shell command. Can stream output to console or capture it."""
    try:
        if stream_output:
            process = subprocess.Popen(
                command,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, # Merge stderr
                text=True,
                bufsize=1,
                universal_newlines=True
            )
            output = []
            # Read char by char to handle \r progress bars
            while True:
                char = process.stdout.read(1)
                if not char and process.poll() is not None:
                    break
                if char:
                    print(char, end='', flush=True)
                    output.append(char)
            
            process.wait()
            # Mock a completed process result
            class Result:
                returncode = process.returncode
                stdout = "".join(output)
                stderr = "" 
            return Result()
        else:
            result = subprocess.run(
                command,
                shell=True,
                check=check,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            return result
    except subprocess.CalledProcessError as e:
        return e

# Helper to find sound files locally or in system
def resolve_sound_path(name):
    # Check if exact filename provided
    local_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), name)
    if os.path.exists(local_path):
        return local_path
    
    # Check with .aiff extension logic
    name_with_ext = name if "." in name else f"{name}.aiff"
    local_path_ext = os.path.join(os.path.dirname(os.path.abspath(__file__)), name_with_ext)
    if os.path.exists(local_path_ext):
        return local_path_ext
        
    # Check .mp3 if not specified
    if "." not in name:
         local_path_mp3 = os.path.join(os.path.dirname(os.path.abspath(__file__)), f"{name}.mp3")
         if os.path.exists(local_path_mp3):
             return local_path_mp3
    
    # Fallback to system sounds (MacOS specific)
    return f"/System/Library/Sounds/{name_with_ext}"

def play_sound(sound):
    """Plays a system sound (non-blocking). Returns process."""
    path = resolve_sound_path(sound)
    return subprocess.Popen(
        f"afplay \"{path}\"",
        shell=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )

def start_looping_sound(sound):
    """Starts a system sound in a loop (non-blocking). Returns process."""
    path = resolve_sound_path(sound)
    # We use a shell loop to keep playing the sound
    return subprocess.Popen(
        f"while true; do afplay \"{path}\"; sleep 0.1; done",
        shell=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid # Sets session ID so we can kill the whole group (shell + sleep/afplay)
    )

def stop_sound(process):
    """Stops a background sound process."""
    if process:
        try:
            # Kill the process group to ensure the shell loop AND the current sleep/afplay child are killed
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        except:
            # Fallback if group kill fails or process already dead
            process.terminate()

def get_probes():
    """Returns a list of connected probe UIDs."""
    result = run_command("pyocd list --no-header")
    if result.returncode != 0:
        return []
    
    uids = []
    for line in result.stdout.splitlines():
        if "No code" in line or not line.strip():
            continue
        # Split by spaces and find parts that look like long hex UIDs (usually 16+ chars)
        parts = line.split()
        for p in parts:
            if len(p) >= 16 and all(c in "0123456789abcdefABCDEF" for c in p):
                uids.append(p)
                break
    return uids


def flash_device(uid, hex_path):
    """Attempts to flash the device with retries."""
    print(f"⚡️ [{uid[:8]}...] Attempting to flash {TARGET}...")
    update_status(uid, "FLASHING")
    
    # Retry loop
    max_retries = 3
    success = False
    
    for attempt in range(max_retries):
        if attempt > 0:
             print(f"   ⚠️ [{uid[:8]}...] Retry {attempt+1}/{max_retries}...")
             time.sleep(0.5)

        # Targeting specific probe with -u
        cmd = f"pyocd flash -u {uid} -t {TARGET} --pack \"{PACK_PATH}\" --frequency 15000000 --erase chip \"{hex_path}\""
        result = run_command(cmd, stream_output=False) # Multi-threaded, stream output would be messy
        
        if result.returncode == 0:
            success = True
            break
    
    if success:
        print(f"✅ [{uid[:8]}...] SUCCESS: Flash verified.")
        update_status(uid, "SUCCESS")
        play_sound("Glass")
        return True
    else:
        print(f"❌ [{uid[:8]}...] ERROR: Flash failed after retries.")
        update_status(uid, "ERROR")
        return False


def list_hex_files():
    """Returns a list of .hex files in the current directory."""
    current_dir = os.path.dirname(os.path.abspath(__file__))
    return sorted([f for f in os.listdir(current_dir) if f.endswith(".hex")])

def select_hex_file():
    """Prompts the user to select a hex file if multiple exist."""
    hex_files = list_hex_files()
    
    if not hex_files:
        print("❌ ERROR: No .hex files found in the current directory.")
        sys.exit(1)
        
    if len(hex_files) == 1:
        selected = hex_files[0]
        print(f"📂 Auto-selected hex: {selected}")
        return os.path.join(os.path.dirname(os.path.abspath(__file__)), selected)
        
    print("\nSelect firmware to flash:")
    for i, f in enumerate(hex_files):
        print(f"  [{i+1}] {f}")
        
    while True:
        try:
            choice = input(f"\nChoose 1-{len(hex_files)}: ")
            index = int(choice) - 1
            if 0 <= index < len(hex_files):
                selected = hex_files[index]
                print(f"📂 Selected hex: {selected}")
                return os.path.join(os.path.dirname(os.path.abspath(__file__)), selected)
            else:
                print("Invalid choice.")
        except ValueError:
            print("Please enter a number.")
        except KeyboardInterrupt:
            print("\nExiting...")
            sys.exit(0)

def start_status_server(port=8000):
    """Starts a simple HTTP server in the background to serve index.html and status.json."""
    class QuietHandler(http.server.SimpleHTTPRequestHandler):
        def log_message(self, format, *args):
            # Suppress logging to keep the terminal clean for flash output
            return

    # Allow port reuse to prevent "Address already in use" on restarts
    socketserver.TCPServer.allow_reuse_address = True
    try:
        with socketserver.TCPServer(("", port), QuietHandler) as httpd:
            print(f"📡 UI Server: http://localhost:{port}")
            httpd.serve_forever()
    except Exception as e:
        print(f"⚠️ Could not start UI server: {e}")

def probe_worker(uid, hex_path):
    """Thread function to manage a single probe's lifecycle."""
    print(f"🧵 [{uid[:8]}...] Started worker thread.")
    update_status(uid, "READY")
    
    check_cmd = f"pyocd cmd -u {uid} -t {TARGET} --pack \"{PACK_PATH}\" -c init -c exit"
    
    while True:
        # Check if probe is still connected
        current_probes = get_probes()
        if uid not in current_probes:
            print(f"🔌 [{uid[:8]}...] Disconnected. Stopping worker.")
            remove_probe_status(uid)
            break

        # Check for TARGET presence
        check_res = run_command(check_cmd)
        check_out = check_res.stdout + check_res.stderr
        
        if check_res.returncode == 0 and "Error" not in check_out and "failure" not in check_out:
            # Target detected!
            print(f"🎯 [{uid[:8]}...] Target detected! Stabilizing...")
            time.sleep(0.5)
            
            if flash_device(uid, hex_path):
                # Success
                pass
            
            # Wait for disconnect
            print(f"⏳ [{uid[:8]}...] Waiting for board release...")
            while True:
                time.sleep(0.2)
                # Quick check for probe loss first
                if uid not in get_probes():
                    break
                    
                check_res_gone = run_command(check_cmd)
                check_out_gone = check_res_gone.stdout + check_res_gone.stderr
                if check_res_gone.returncode != 0 or "Error" in check_out_gone or "failure" in check_out_gone:
                    print(f"🔓 [{uid[:8]}...] Board released.")
                    update_status(uid, "READY")
                    time.sleep(0.2)
                    break
        else:
            # No target, keep waiting
            time.sleep(0.5)

def main():
    # Start status server in a daemon thread
    threading.Thread(target=start_status_server, daemon=True).start()
    
    # Wait a moment for server to start then open browser
    time.sleep(1.0)
    print("🌍 Opening status UI in browser...")
    webbrowser.open("http://localhost:8000")

    hex_path = select_hex_file()
    
    print("---------------------------------------")
    print(f"🚀 MULTI-FLASH: {TARGET}")
    print(f"📂 HEX: {os.path.basename(hex_path)}")
    print("---------------------------------------")
    
    active_threads = {} # uid -> Thread

    while True:
        try:
            current_uids = get_probes()
            
            # Start new threads for new probes
            for uid in current_uids:
                if uid not in active_threads or not active_threads[uid].is_alive():
                    t = threading.Thread(target=probe_worker, args=(uid, hex_path), daemon=True)
                    active_threads[uid] = t
                    t.start()
            
            # Clean up old thread references (worker exits itself on disconnect)
            dead_uids = [uid for uid, t in active_threads.items() if not t.is_alive()]
            for uid in dead_uids:
                del active_threads[uid]
                
            if not active_threads:
                # Use \r to stay on one line while waiting
                print("\r🔍 Searching for probes...", end="", flush=True)
            
            time.sleep(1.0)
        except KeyboardInterrupt:
            break


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nExiting...")
        sys.exit(0)
