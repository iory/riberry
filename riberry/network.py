import os
import platform
import socket
import subprocess
import time


def get_wifi_info():
    try:
        result = subprocess.check_output(["iwconfig"], text=True, stderr=subprocess.DEVNULL)
        ssid = "unknown"
        rssi = "N/A"
        for line in result.splitlines():
            if "ESSID" in line:
                ssid = line.split("ESSID:")[1].strip().strip('"')
            if "Signal level" in line:
                rssi = line.split("Signal level=")[1].split(" ")[0]
        return ssid, rssi
    except Exception:
        return None, None


def parse_ip(route_get_output):
    tokens = route_get_output.split()
    if "via" in tokens:
        return tokens[tokens.index("via") + 5]
    else:
        return tokens[tokens.index("src") + 1]


def get_ros_ip():
    try:
        route_get = subprocess.check_output(
            ["ip", "-o", "route", "get", "8.8.8.8"], stderr=subprocess.DEVNULL
        ).decode()
        return parse_ip(route_get)
    except subprocess.CalledProcessError:
        return None


def wait_and_get_ros_ip(retry=300):
    for _ in range(retry):
        ros_ip = get_ros_ip()
        if ros_ip:
            return ros_ip
        time.sleep(1)
    return None


def get_mac_address(interface=None):
    if interface is None:
        if platform.system() == "Darwin":  # macOS
            interface = "en0"
        else:
            interface = "wlan0"
    try:
        if platform.system() == "Darwin":
            mac_address = (
                subprocess.check_output(["ifconfig", interface])
                .decode("utf-8")
                .split("ether ")[1]
                .split()[0]
            )
        else:
            mac_address = (
                subprocess.check_output(["cat", f"/sys/class/net/{interface}/address"])
                .decode("utf-8")
                .strip()
            )
        mac_address = mac_address.replace(":", "")
        return mac_address
    except Exception as e:
        print(f"Error obtaining MAC address: {e}")
        return None


def get_ip_address():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
    except OSError as e:
        print(e)
    ip_address = s.getsockname()[0]
    s.close()
    if ip_address == "0.0.0.0":
        return None
    return ip_address


def get_ros_master_ip():
    master_str = os.getenv("ROS_MASTER_URI", default="None")
    # https://[IP Address]:11311 -> [IP Address]
    master_str = master_str.split(":")
    if len(master_str) > 1:
        master_ip = master_str[1].replace("/", "")
    else:
        return "none"
    return master_ip
