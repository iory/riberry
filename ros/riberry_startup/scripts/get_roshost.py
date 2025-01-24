#!/usr/bin/env python3

import argparse
import socket
import subprocess
import time


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


def get_roshost(retry=None, ros_master_ip=None):
    ros_ip = wait_and_get_ros_ip(retry or 300)
    ros_script = ""
    if ros_ip:
        ros_script += f"ROS_IP={ros_ip}"
    else:
        ros_script += f"ROS_HOSTNAME={socket.gethostname()}.local"
    if ros_master_ip is not None:
        ros_script += f" ROS_MASTER_URI=http://{ros_master_ip}:11311"
    return ros_script


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='IP Address Parser')
    parser.add_argument(
        '-', '--rossetmaster', type=str, nargs='?', const=None,
        help='IP address to display (default: 8.8.8.8)')
    args = parser.parse_args()

    roshost = get_roshost(ros_master_ip=args.rossetmaster)
    print(roshost)
