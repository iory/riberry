#!/usr/bin/env python3

import subprocess
import time
import socket


def parse_ip(route_get_output):
    tokens = route_get_output.split()
    if "via" in tokens:
        return tokens[tokens.index("via") + 5]
    else:
        return tokens[tokens.index("src") + 1]


def get_ros_ip():
    try:
        route_get = subprocess.check_output(
            ["ip", "-o", "route", "get", "8.8.8.8"],
            stderr=subprocess.DEVNULL).decode()
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


def get_roshost(retry=None):
    ros_ip = wait_and_get_ros_ip(retry or 300)
    if ros_ip:
        return f"ROS_IP={ros_ip}"
    else:
        return f"ROS_HOSTNAME={socket.gethostname()}.local"


if __name__ == "__main__":
    roshost = get_roshost()
    print(roshost)
