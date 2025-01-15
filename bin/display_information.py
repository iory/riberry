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
from riberry.com.base import ComBase
from riberry.com.base import PacketType
from riberry.com.i2c_base import I2CBase
from riberry.com.uart_base import UARTBase
from riberry.network import get_ip_address
from riberry.network import get_mac_address
from riberry.network import get_ros_master_ip
from riberry.network import wait_and_get_ros_ip

# Ensure that the standard output is line-buffered. This makes sure that
# each line of output is flushed immediately, which is useful for logging.
# This is for systemd.
sys.stdout.reconfigure(line_buffering=True)


battery_readers = []

# Global variable for ROS availability and additional message
ros_available = False
ros_additional_message = None
atom_s3_mode = "DisplayInformationMode"
button_count = 0
ros_display_image_flag = False
ros_display_image = None
stop_event = threading.Event()
wifi_connected = False


def try_init_ros():
    global ros_available
    global ros_additional_message
    global ros_display_image_flag
    global ros_display_image
    global battery_readers
    global atom_s3_mode
    global button_count
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

            def ros_image_callback(msg):
                global ros_display_image
                bridge = cv_bridge.CvBridge()
                ros_display_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            rospy.init_node("display_information", anonymous=False)
            rospy.Subscriber(
                "/atom_s3_additional_info", String, ros_callback, queue_size=1
            )
            mode_pub = rospy.Publisher("/atom_s3_mode", String, queue_size=1)
            button_pub = rospy.Publisher("/atom_s3_button_state", Int32, queue_size=1)
            for battery_reader in battery_readers:
                if isinstance(battery_reader, MP2760BatteryMonitor):
                    battery_pub = rospy.Publisher(
                        "/battery/remaining_battery", Float32, queue_size=1
                    )
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
                elif isinstance(battery_reader, PisugarBatteryReader):
                    pisugar_battery_pub = rospy.Publisher(
                        "/battery/pisugar/remaining_battery", Float32, queue_size=1
                    )

            ros_available = True
            rate = rospy.Rate(10)
            sub = None
            while not rospy.is_shutdown() and not stop_event.is_set():
                for battery_reader in battery_readers:
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
                        pisugar_battery_pub.publish(battery_reader.get_filtered_percentage())

                mode_pub.publish(String(data=atom_s3_mode))
                button_pub.publish(Int32(data=button_count))
                button_count = 0
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


class DisplayInformation:
    def __init__(self):
        device = ComBase.identify_device()
        if device == 'm5stack-LLM':
            self.com = UARTBase()
        else:
            self.com = I2CBase(0x42)

    def display_image(self, img):
        img = squared_padding_image(img, 128)
        quality = 75
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        result, jpg_img = cv2.imencode(".jpg", img, encode_param)
        jpg_size = len(jpg_img)

        header = []
        header += [PacketType.JPEG]
        header += [(jpg_size & 0xFF00) >> 8, (jpg_size & 0x00FF) >> 0]
        self.com.write(header)

        time.sleep(0.005)

        for pack in nsplit(jpg_img, n=50):
            self.com.write([PacketType.JPEG] + pack.tolist())
            time.sleep(0.005)

    def display_information(self):
        global ros_available
        global ros_additional_message
        global battery_readers

        ip = get_ip_address()
        if ip is None:
            ip = "no connection"
        ip_str = f"{socket.gethostname()}:\n{Fore.YELLOW}{ip}{Fore.RESET}"
        master_str = "ROS_MASTER:\n" + Fore.RED + f"{get_ros_master_ip()}" + Fore.RESET
        battery_str = ""
        if battery_readers:
            for i, battery_reader in enumerate(battery_readers):
                if i > 0:
                    battery_str += '\n'
                if isinstance(battery_reader, PisugarBatteryReader):
                    prefix = 'PCBat'
                else:
                    prefix = 'ServoBat'
                charging = battery_reader.get_is_charging()
                battery = battery_reader.get_filtered_percentage()
                if battery is None:
                    battery_str += "{prefix}: None"
                else:
                    if battery <= 20:
                        battery_str += f"{prefix}: {Fore.RED}{int(battery)}%{Fore.RESET}"
                    else:
                        battery_str += f"{prefix}: {Fore.GREEN}{int(battery)}%{Fore.RESET}"
                if charging is True:
                    battery_str += "+"
                elif charging is False:
                    battery_str += "-"
                else:
                    battery_str += "?"
        sent_str = [chr(PacketType.TEXT)]
        sent_str += f"{ip_str}\n{master_str}\n{battery_str}\n"

        if ros_available and ros_additional_message:
            sent_str += f"{ros_additional_message}\n"
            ros_additional_message = None

        self.com.write(sent_str)

    def display_qrcode(self, target_url=None):
        header = [PacketType.QR_CODE]
        if target_url is None:
            ip = get_ip_address()
            if ip is None:
                print("Could not get ip. skip showing qr code.")
                return
            target_url = f"http://{ip}:8085/riberry_startup/"
        header += [len(target_url)]
        header += list(map(ord, target_url))
        self.com.write(header)

    def force_mode(self, mode_name):
        header = [PacketType.FORCE_MODE]
        forceModebytes = (list (map(ord, mode_name)))
        self.com.write(header + forceModebytes)

    def run(self):
        global ros_display_image
        global ros_display_image_flag
        global atom_s3_mode
        global wifi_connected
        global button_count
        ssid = f'{self.com.identify_device()}-{get_mac_address()}'
        ssid = ssid.replace(' ', '-')
        qrcode_mode_is_forced = False

        while not stop_event.is_set():
            try:
                self.com.read()
                self.com.write([PacketType.BUTTON_STATE_REQUEST])
                time.sleep(0.1)
                mode_packet = self.com.read()
                if len(mode_packet) > 1:
                    button_count = int(mode_packet[0])
                    atom_s3_mode = mode_packet[1:].decode(errors="ignore")
            except Exception as e:
                print(f"Mode reading failed. {e}")
            mode = atom_s3_mode
            print(f"Mode: {mode} device_type: {self.com.device_type}")
            # Display the QR code when Wi-Fi is not connected,
            # regardless of atom_s3_mode.
            if get_ip_address() is None:
                wifi_connected = False
                if qrcode_mode_is_forced is False:
                    self.force_mode("DisplayQRcodeMode")
                    qrcode_mode_is_forced = True
                    time.sleep(1)
                self.display_qrcode(f"WIFI:S:{ssid};T:nopass;;")
                time.sleep(1)
                continue
            else:
                if wifi_connected is False:
                    self.force_mode("DisplayInformationMode")
                wifi_connected = True
            # Display data according to mode
            if mode == "DisplayInformationMode":
                self.display_information()
                time.sleep(0.1)
            elif mode == "DisplayQRcodeMode":
                self.display_qrcode()
                time.sleep(0.1)
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
        battery_readers.append(MP2760BatteryMonitor(battery_bus_number))
    if PisugarBatteryReader.exists(battery_bus_number):
        print("[Display Information] Use Pisugar")
        battery_readers.append(PisugarBatteryReader(battery_bus_number))
    for battery_reader in battery_readers:
        battery_reader.daemon = True
        battery_reader.start()

    display_thread = threading.Thread(target=DisplayInformation().run)
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
