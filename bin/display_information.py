#!/usr/bin/env python3

import os
import socket
import subprocess
import sys
import threading
import time

from colorama import Fore
import cv2
from i2c_for_esp32 import WirePacker
from pybsc import nsplit
from pybsc.image_utils import squared_padding_image

from riberry.battery import MP2760BatteryMonitor
from riberry.battery import PisugarBatteryReader
from riberry.i2c_base import I2CBase

# Ensure that the standard output is line-buffered. This makes sure that
# each line of output is flushed immediately, which is useful for logging.
# This is for systemd.
sys.stdout.reconfigure(line_buffering=True)


pisugar_battery_percentage = None
debug_i2c_text = False


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


def get_mac_address(interface="wlan0"):
    try:
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


# Global variable for ROS availability and additional message
ros_available = False
ros_additional_message = None
atom_s3_mode = None
ros_display_image_flag = False
ros_display_image = None
stop_event = threading.Event()


def try_init_ros():
    global ros_available
    global ros_additional_message
    global ros_display_image_flag
    global ros_display_image
    global pisugar_battery_percentage
    ros_display_image_param = None
    prev_ros_display_image_param = None
    while not stop_event.is_set():
        try:
            import cv_bridge
            import rospy
            import sensor_msgs.msg
            from std_msgs.msg import Float32
            from std_msgs.msg import String

            ros_ip = wait_and_get_ros_ip(300)
            print(f"Set ROS_IP={ros_ip}")
            os.environ["ROS_IP"] = ros_ip

            def ros_callback(msg):
                global ros_additional_message
                ros_additional_message = msg.data

            def ros_mode_callback(msg):
                global atom_s3_mode
                atom_s3_mode = msg.data

            def ros_image_callback(msg):
                global ros_display_image
                bridge = cv_bridge.CvBridge()
                ros_display_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            rospy.init_node("atom_s3_display_information_node", anonymous=True)
            rospy.Subscriber(
                "/atom_s3_additional_info", String, ros_callback, queue_size=1
            )
            rospy.Subscriber("/atom_s3_mode", String, ros_mode_callback, queue_size=1)
            battery_pub = rospy.Publisher("/pisugar_battery", Float32, queue_size=1)
            ros_available = True
            rate = rospy.Rate(1)
            sub = None
            while not rospy.is_shutdown() and not stop_event.is_set():
                ros_display_image_param = rospy.get_param("/display_image", None)
                if pisugar_battery_percentage is not None:
                    battery_pub.publish(pisugar_battery_percentage)
                if prev_ros_display_image_param != ros_display_image_param:
                    ros_display_image_flag = False
                    if sub is not None:
                        sub.unregister()
                        sub = None
                    if ros_display_image_param:
                        rospy.loginfo(
                            f"Start subscribe {ros_display_image_param} for display"
                        )
                        ros_display_image_flag = True
                        sub = rospy.Subscriber(
                            ros_display_image_param,
                            sensor_msgs.msg.Image,
                            queue_size=1,
                            callback=ros_image_callback,
                        )
                prev_ros_display_image_param = ros_display_image_param
                rate.sleep()
            if rospy.is_shutdown():
                break
        except ImportError as e:
            print(f"ROS is not available ({e}). Retrying...")
            time.sleep(5)  # Wait before retrying
        except rospy.ROSInterruptException as e:
            print(f"ROS interrupted ({e}). Retrying...")
            time.sleep(5)  # Wait before retrying
        finally:
            ros_available = False
            ros_additional_message = None


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


class DisplayInformation(I2CBase):
    def __init__(self, i2c_addr):
        super().__init__(i2c_addr)
        use_pisugar = False

        if MP2760BatteryMonitor.exists(self.bus_number):
            print("[Display Information] Use JSK Battery Board")
        else:
            print("[Display Information] Use Pisugar")
            use_pisugar = True

        self.use_pisugar = use_pisugar
        if self.bus_number:
            if use_pisugar:
                self.battery_reader = PisugarBatteryReader(self.bus_number)
            else:
                self.battery_reader = MP2760BatteryMonitor(self.bus_number)
            self.battery_reader.daemon = True
            self.battery_reader.start()
        else:
            self.battery_reader = None

    def display_image(self, img):
        img = squared_padding_image(img, 128)
        quality = 75
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        result, jpg_img = cv2.imencode(".jpg", img, encode_param)
        jpg_size = len(jpg_img)

        header = []
        header += [0xFF, 0xD8, 0xEA]
        header += [(jpg_size & 0xFF00) >> 8, (jpg_size & 0x00FF) >> 0]

        packer = WirePacker(buffer_size=1000)
        for h in header:
            packer.write(h)
        packer.end()

        if packer.available():
            self.i2c_write(packer.buffer[: packer.available()])

        time.sleep(0.005)

        for pack in nsplit(jpg_img, n=50):
            packer.reset()
            for h in [0xFF, 0xD8, 0xEA]:
                packer.write(h)
            for h in pack:
                packer.write(h)
            packer.end()
            if packer.available():
                self.i2c_write(packer.buffer[: packer.available()])
            time.sleep(0.005)

    def display_information(self):
        global ros_available
        global ros_additional_message
        global pisugar_battery_percentage

        ip = get_ip_address()
        if ip is None:
            ip = "no connection"
        ip_str = f"{socket.gethostname()}:\n{Fore.YELLOW}{ip}{Fore.RESET}"
        master_str = "ROS_MASTER:\n" + Fore.RED + f"{get_ros_master_ip()}" + Fore.RESET
        battery_str = ""
        if self.battery_reader:
            charging = self.battery_reader.get_is_charging()
            battery = self.battery_reader.get_filtered_percentage(charging)
            pisugar_battery_percentage = battery
            if battery is None:
                battery_str = "Bat: None"
            else:
                if battery <= 20:
                    battery_str = f"Bat: {Fore.RED}{int(battery)}%{Fore.RESET}"
                else:
                    battery_str = f"Bat: {Fore.GREEN}{int(battery)}%{Fore.RESET}"
            if charging is True:
                battery_str += "+"
            elif charging is False:
                battery_str += "-"
            else:
                battery_str += "?"
        sent_str = f"{ip_str}\n{master_str}\n{battery_str}\n"

        if ros_available and ros_additional_message:
            sent_str += f"{ros_additional_message}\n"
            ros_additional_message = None

        if debug_i2c_text:
            print("send the following message")
            print(sent_str)
        self.send_string(sent_str)

    def display_qrcode(self, target_url=None):
        header = [0x02]
        if target_url is None:
            ip = get_ip_address()
            if ip is None:
                print("Could not get ip. skip showing qr code.")
                return
            target_url = f"http://{ip}:8085/riberry_startup/"
        header += [len(target_url)]
        header += list(map(ord, target_url))
        print("header")
        print(header)
        self.send_raw_bytes(header)

    def run(self):
        global ros_display_image
        global ros_display_image_flag
        global atom_s3_mode

        while not stop_event.is_set():
            mode = atom_s3_mode
            print(f"Mode: {mode} device_type: {self.device_type}")
            if mode != "DisplayInformationMode" and mode != "DisplayQRcodeMode":
                time.sleep(0.1)
                continue
            if ros_display_image_flag and ros_display_image is not None:
                self.display_image(ros_display_image)
            else:
                if get_ip_address() is None:
                    if self.device_type == "Raspberry Pi":
                        ssid = f"raspi-{get_mac_address()}"
                    elif self.device_type == "Radxa Zero":
                        ssid = f"radxa-{get_mac_address()}"
                    else:
                        ssid = f"radxa-{get_mac_address()}"
                    self.display_qrcode(f"WIFI:S:{ssid};T:nopass;;")
                    time.sleep(3)
                else:
                    if mode == "DisplayInformationMode":
                        self.display_information()
                        time.sleep(3)
                    elif mode == "DisplayQRcodeMode":
                        self.display_qrcode()
                        time.sleep(3)
                    else:
                        time.sleep(3)


if __name__ == "__main__":
    display_thread = threading.Thread(target=DisplayInformation(0x42).run)
    display_thread.daemon = True
    display_thread.start()

    try:
        try_init_ros()
    except KeyboardInterrupt:
        print("Interrupted by user, shutting down...")
        stop_event.set()
        display_thread.join()
