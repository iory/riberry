#!/usr/bin/env python3

import os.path as osp
from pathlib import Path
import subprocess
import time

# Wait for 30 seconds to allow network services to initialize
time.sleep(30)


# Function to get the SSID of the connected WiFi network (if any)
def get_ssid():
    try:
        ssid = subprocess.check_output(["iwgetid", "-r"]).decode().strip()
        return ssid if ssid else None
    except subprocess.CalledProcessError:
        return None


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
ssid = get_ssid()

if ssid:
    print(f"Skipping WiFi Connect. Connected to SSID: {ssid}")
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
