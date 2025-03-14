#!/usr/bin/env python3

import os.path as osp
from pathlib import Path
import subprocess
import sys
import time

# Ensure that the standard output is line-buffered. This makes sure that
# each line of output is flushed immediately, which is useful for logging.
# This is for systemd.
sys.stdout.reconfigure(line_buffering=True)

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

if not is_nm_ready():
    print("NetworkManager not ready. Exiting.")
    sys.exit(1)

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

def is_wifi_connected(interface="wlan0"):
    while not exists_interface(interface):
        print(f"Waiting for {interface} to be appeared...")
        time.sleep(1)
    try:
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

# Function to get the MAC address of wlan0
def get_mac_address():
    try:
        mac_address = Path("/sys/class/net/wlan0/address").read_text().strip()
        return mac_address.replace(":", "") if mac_address else None
    except FileNotFoundError:
        return None


# Check if connected to a WiFi network
if is_wifi_connected():
    print("Skipping WiFi Connect.")
else:
    print("No default gateway found. Starting WiFi Connect.")
    model = identify_device().replace(" ", "-")
    mac_address = get_mac_address()
    if mac_address:
        # Run wifi-connect with the specified parameters
        try:
            result = subprocess.run(
                [
                    "wifi-connect",
                    "-s",
                    f"{model}-{mac_address}",
                    "-g",
                    "192.168.4.1",
                    "-d",
                    "192.168.4.2,192.168.4.5",
                ],
                check=True,
            )
            print("WiFi Connect started successfully.")
        except subprocess.CalledProcessError:
            print("WiFi Connect failed to start.")
    else:
        print("MAC address not found. WiFi Connect could not start.")
