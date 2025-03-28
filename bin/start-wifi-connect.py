#!/usr/bin/env python3

import os.path as osp
from pathlib import Path
import socket
import subprocess
import sys
import time

# Ensure that the standard output is line-buffered. This makes sure that
# each line of output is flushed immediately, which is useful for logging.
# This is for systemd.
sys.stdout.reconfigure(line_buffering=True)

# Abstract Unix socket setup
SOCKET_PATH = '\0wifi_connect'
server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
server.bind(SOCKET_PATH)
server.listen(1)

wifi_connect_process = None


def is_connecting(interface="wlan0", timeout=10):
    start_time = time.time()
    print(f"Checking connection status for {interface} (timeout: {timeout}s)...")
    while time.time() - start_time < timeout:
        status = get_nm_connection_status()
        if status is not None:
            current_state = status['state']
            connectivity = status['connectivity']
            elapsed_time = round(time.time() - start_time, 1)
            if current_state == 'connected':
                print(f"[{elapsed_time}s] Connection established (connectivity: {connectivity})")
                break
            elif current_state == 'connecting':
                print(f"[{elapsed_time}s] Still connecting... (connectivity: {connectivity})")
            elif current_state == 'disconnected':
                print(f"[{elapsed_time}s] Disconnected, waiting... (connectivity: {connectivity})")
            else:
                print(f"[{elapsed_time}s] State: {current_state}, connectivity: {connectivity}")
        else:
            elapsed_time = round(time.time() - start_time, 1)
            print(f"[{elapsed_time}s] Unable to retrieve status, retrying...")
        time.sleep(1.0)
    if status and status['state'] == 'connected':
        return False
    elif status and status['state'] == 'connecting':
        print(f"Timeout after {timeout}s, still in connecting state")
        return True
    else:
        print(f"Timeout after {timeout}s, not connected")
        return False

def get_nm_connection_status():
    try:
        result = subprocess.run(
            ["nmcli", "-t", "-f", "STATE,CONNECTIVITY", "general", "status"],
            capture_output=True,
            text=True,
            check=True
        )
        status_output = result.stdout.strip().split(':')
        if len(status_output) >= 2:
            state = status_output[0]  # connected, connecting, disconnected
            connectivity = status_output[1]  # full, limited, none
            return {
                "state": state,
                "connectivity": connectivity
            }
        else:
            print("Unexpected nmcli general status output")
            return None
    except subprocess.CalledProcessError as e:
        print(f"Error getting NetworkManager status: {e}")
        return None
    except Exception as e:
        print(f"Unexpected error: {e}")
        return None

def is_nm_ready(timeout=10):
    for _ in range(timeout):
        result = subprocess.run(
            ["nmcli", "-t", "-f", "RUNNING", "general"],
            capture_output=True, text=True
        )
        if result.stdout.strip() == "running":
            return True
        print("Waiting for NetworkManager to become ready...")
        time.sleep(1)
    return False

def exists_interface(interface):
    result = subprocess.run(
        ["ip", "link", "show", interface],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True
    )
    if result.returncode == 0:
        print(f"Interface {interface} found.")
        return True
    print(f"Waiting for interface {interface} to appear...")
    return False

def is_wifi_connected(interface="wlan0", timeout=10):
    while not exists_interface(interface):
        print(f"Waiting for {interface} to be appeared...")
        time.sleep(1)
    try:
        is_connecting()

        # Check if the interface has an IP address
        result = subprocess.run(
            ["ip", "addr", "show", interface],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True
        )
        if "inet " in result.stdout:
            # Get the SSID of the connected network
            ssid_result = subprocess.run(
                ["iwgetid", "-r"],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True
            )
            ssid = ssid_result.stdout.strip()
            print(f"Connected to SSID: {ssid}")
            return True
        else:
            print("No IP address assigned to the interface.")
            return False
    except Exception as e:
        print(f"Error checking Wi-Fi connection: {e}")
        return False


def identify_device():
    try:
        with open("/proc/cpuinfo") as f:
            cpuinfo = f.read()
        if "Raspberry Pi" in cpuinfo:
            return "Raspberry Pi"
        if osp.exists("/proc/device-tree/model"):
            with open("/proc/device-tree/model") as f:
                model = f.read().strip().replace("\x00", "")
            if "Radxa" in model or "ROCK Pi" in model\
               or model == "Khadas VIM4"\
               or model == "NVIDIA Jetson Xavier NX Developer Kit":
                return model
        if osp.exists('/usr/local/m5stack/block-mount.sh'):
            return 'm5stack-LLM'
        return "Unknown Device"
    except FileNotFoundError:
        return "Unknown Device"

def get_mac_address(interface='wlan0'):
    try:
        mac_address = Path(f"/sys/class/net/{interface}/address").read_text().strip()
        return mac_address.replace(":", "") if mac_address else None
    except FileNotFoundError:
        return None

def stop_wifi_connect():
    global wifi_connect_process
    if wifi_connect_process and wifi_connect_process.poll() is None:
        wifi_connect_process.terminate()
        wifi_connect_process.wait()
        print("WiFi Connect stopped.")
    wifi_connect_process = None

def start_wifi_connect(model, mac_address):
    global wifi_connect_process
    if wifi_connect_process and wifi_connect_process.poll() is None:
        print("wifi-connect is already running.")
        return
    try:
        wifi_connect_process = subprocess.Popen(
            ["/usr/local/sbin/wifi-connect", "-s", f"{model}-{mac_address}", "-g", "192.168.4.1", "-d", "192.168.4.2,192.168.4.5"]
        )
        print("WiFi Connect started successfully.")
    except Exception as e:
        print(f"Failed to start wifi-connect: {e}")

while not is_nm_ready():
    print("NetworkManager not ready. Waiting...")
    time.sleep(1.0)

model = identify_device().replace(" ", "-")
mac_address = get_mac_address()
while not mac_address:
    print("MAC address not found. Waiting...")
    time.sleep(1.0)
    model = identify_device().replace(" ", "-")
    mac_address = get_mac_address()

if is_wifi_connected():
    print("Skipping WiFi Connect.")
else:
    print("No default gateway found. Starting WiFi Connect.")
    start_wifi_connect(model, mac_address)

# Main loop to listen for commands
print("Waiting for commands on abstract socket...")
while True:
    conn, addr = server.accept()
    command = conn.recv(1024).decode().strip()

    if command == "switch_wifi":
        print("Received command: Switch to another Wi-Fi")
        start_wifi_connect(model, mac_address)
    elif command == "restart_wifi_connect":
        print("Received command: Restart wifi-connect")
        start_wifi_connect(model, mac_address)
    elif command == "stop_wifi_connect":
        print("Received command: Stop wifi-connect")
        stop_wifi_connect()
    elif command == "status":
        # Check wifi-connect status and respond
        status = "running" if wifi_connect_process and wifi_connect_process.poll() is None else "stopped"
        print(f"Received command: Check status - {status}")
        conn.send(status.encode())
        conn.close()
    else:
        print(f"Unknown command received: {command}")
