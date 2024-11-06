#!/usr/bin/env python3

import os
import socket
import sys
import threading
import time

from colorama import Fore
import cv2
from i2c_for_esp32 import WirePacker
from pybsc import nsplit
from pybsc.image_utils import squared_padding_image

from riberry.battery import decide_battery_i2c_bus_number
from riberry.battery import MP2760BatteryMonitor
from riberry.battery import PisugarBatteryReader
from riberry.i2c_base import I2CBase
from riberry.network import get_ip_address
from riberry.network import get_mac_address
from riberry.network import get_ros_master_ip
from riberry.network import wait_and_get_ros_ip

# Ensure that the standard output is line-buffered. This makes sure that
# each line of output is flushed immediately, which is useful for logging.
# This is for systemd.
sys.stdout.reconfigure(line_buffering=True)


battery_reader = None

# Global variable for ROS availability and additional message
ros_available = False
ros_additional_message = None
atom_s3_mode = "DisplayInformationMode"
ros_display_image_flag = False
ros_display_image = None
stop_event = threading.Event()


def try_init_ros():
    global ros_available
    global ros_additional_message
    global ros_display_image_flag
    global ros_display_image
    global battery_reader
    ros_display_image_param = None
    prev_ros_display_image_param = None

    battery_junction_temperature = None
    input_voltage = None
    system_voltage = None
    charge_status = None
    battery_charge_current = None
    status_and_fault = None
    status_and_fault_string = None
    battery_percentage = None
    while not stop_event.is_set():
        try:
            import cv_bridge
            import rospy
            import sensor_msgs.msg
            from std_msgs.msg import Float32
            from std_msgs.msg import Int32
            from std_msgs.msg import String
            from std_msgs.msg import UInt32

            ros_ip = wait_and_get_ros_ip(300)
            if ros_ip is None:
                print('Could not get ros ip. retry.')
                continue
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
            battery_pub = rospy.Publisher(
                "/battery/remaining_battery", Float32, queue_size=1
            )
            if isinstance(battery_reader, MP2760BatteryMonitor):
                battery_temperature_pub = rospy.Publisher(
                    "/battery/junction_temperature", Float32, queue_size=1
                )
                input_voltage_pub = rospy.Publisher(
                    "/battery/input_voltage", Float32, queue_size=1
                )
                system_voltage_pub = rospy.Publisher(
                    "/battery/system_voltage", Float32, queue_size=1
                )
                battery_charge_current_pub = rospy.Publisher(
                    "/battery/battery_charge_current", Float32, queue_size=1
                )
                charge_status_pub = rospy.Publisher(
                    "/battery/charge_status", Int32, queue_size=1
                )
                charge_status_string_pub = rospy.Publisher(
                    "/battery/charge_status_string", String, queue_size=1
                )
                status_and_fault_pub = rospy.Publisher(
                    "/battery/status_and_fault", UInt32, queue_size=1
                )
                status_and_fault_string_pub = rospy.Publisher(
                    "/battery/status_and_fault_string", String, queue_size=1
                )
            ros_available = True
            rate = rospy.Rate(1)
            sub = None
            while not rospy.is_shutdown() and not stop_event.is_set():
                if isinstance(battery_reader, MP2760BatteryMonitor):
                    battery_junction_temperature = battery_reader.junction_temperature
                    input_voltage = battery_reader.input_voltage
                    system_voltage = battery_reader.system_voltage
                    charge_status = battery_reader.charge_status
                    battery_charge_current = battery_reader.battery_charge_current
                    status_and_fault = battery_reader.status_and_fault
                    status_and_fault_string = battery_reader.status_and_fault_string
                    battery_percentage = battery_reader.get_filtered_percentage()
                elif isinstance(battery_reader, PisugarBatteryReader):
                    battery_percentage = battery_reader.get_filtered_percentage()

                ros_display_image_param = rospy.get_param("/display_image", None)
                if battery_percentage is not None:
                    battery_pub.publish(battery_percentage)
                if input_voltage is not None:
                    input_voltage_pub.publish(input_voltage)
                if system_voltage is not None:
                    system_voltage_pub.publish(system_voltage)
                if charge_status is not None:
                    charge_status_pub.publish(int(charge_status.value))
                    charge_status_string_pub.publish(str(charge_status))
                if battery_charge_current is not None:
                    battery_charge_current_pub.publish(battery_charge_current)
                if battery_junction_temperature is not None:
                    battery_temperature_pub.publish(battery_junction_temperature)
                if status_and_fault is not None:
                    status_and_fault_pub.publish(status_and_fault)
                if status_and_fault_string is not None:
                    status_and_fault_string_pub.publish(status_and_fault_string)
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


class DisplayInformation(I2CBase):
    def __init__(self, i2c_addr):
        super().__init__(i2c_addr)

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
        global battery_reader

        ip = get_ip_address()
        if ip is None:
            ip = "no connection"
        ip_str = f"{socket.gethostname()}:\n{Fore.YELLOW}{ip}{Fore.RESET}"
        master_str = "ROS_MASTER:\n" + Fore.RED + f"{get_ros_master_ip()}" + Fore.RESET
        battery_str = ""
        if battery_reader:
            charging = battery_reader.get_is_charging()
            battery = battery_reader.get_filtered_percentage()
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

    def force_mode(self, mode_name):
        header = [0xFF, 0xFE, 0xFD]
        forceModebytes = (list (map(ord, mode_name)))
        self.send_raw_bytes(header + forceModebytes)

    def run(self):
        global ros_display_image
        global ros_display_image_flag
        global atom_s3_mode
        ssid = f'{self.identify_device()}-{get_mac_address()}'
        ssid = ssid.replace(' ', '-')
        qrcode_mode_is_forced = False

        while not stop_event.is_set():
            mode = atom_s3_mode
            print(f"Mode: {mode} device_type: {self.device_type}")
            # Display the QR code when Wi-Fi is not connected,
            # regardless of atom_s3_mode.
            if get_ip_address() is None:
                if qrcode_mode_is_forced is False:
                    self.force_mode("DisplayQRcodeMode")
                    qrcode_mode_is_forced = True
                    time.sleep(1)
                self.display_qrcode(f"WIFI:S:{ssid};T:nopass;;")
                time.sleep(3)
                continue
            # Display data according to mode
            if mode == "DisplayInformationMode":
                self.display_information()
                time.sleep(3)
            elif mode == "DisplayQRcodeMode":
                self.display_qrcode()
                time.sleep(3)
            elif mode == "DisplayImageMode":  # not implemented
                if ros_display_image_flag and ros_display_image is not None:
                    self.display_image(ros_display_image)
                    # Set ros_display_image to None
                    # after displaying the image to ensure it's not reused
                    ros_display_image = None
            else:
                time.sleep(0.1)


if __name__ == "__main__":
    battery_bus_number = decide_battery_i2c_bus_number()
    if MP2760BatteryMonitor.exists(battery_bus_number):
        print("[Display Information] Use JSK Battery Board")
        battery_reader = MP2760BatteryMonitor(battery_bus_number)
    else:
        print("[Display Information] Use Pisugar")
        battery_reader = PisugarBatteryReader(battery_bus_number)
    battery_reader.daemon = True
    battery_reader.start()

    display_thread = threading.Thread(target=DisplayInformation(0x42).run)
    display_thread.daemon = True
    display_thread.start()

    try:
        try_init_ros()
    except KeyboardInterrupt:
        print("Interrupted by user, shutting down...")
        stop_event.set()
        display_thread.join()
    except Exception as e:
        print(f"Error {e}, shutting down...")
        stop_event.set()
        display_thread.join()
