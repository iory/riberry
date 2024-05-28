#!/usr/bin/env python3

import os
import socket
import subprocess
import time
import threading

import cv2
import board
import busio
from colorama import Fore
from i2c_for_esp32 import WirePacker
from pybsc.image_utils import squared_padding_image
from pybsc import nsplit
from filelock import FileLock
from filelock import Timeout


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


def get_mac_address(interface='wlan0'):
    try:
        mac_address = subprocess.check_output(
            ['cat', f'/sys/class/net/{interface}/address']).decode(
                'utf-8').strip()
        mac_address = mac_address.replace(':', '')
        return mac_address
    except Exception as e:
        print(f"Error obtaining MAC address: {e}")
        return None


lock_path = '/tmp/i2c-1.lock'

# Global variable for ROS availability and additional message
ros_available = False
ros_additional_message = None
ros_display_image_flag = False
ros_display_image = None
stop_event = threading.Event()


def try_init_ros():
    global ros_available
    global ros_additional_message
    global ros_display_image_flag
    global ros_display_image
    ros_display_image_param = None
    prev_ros_display_image_param = None
    while not stop_event.is_set():
        try:
            import rospy
            from std_msgs.msg import String
            import sensor_msgs.msg
            import cv_bridge

            ros_ip = wait_and_get_ros_ip(300)
            print('Set ROS_IP={}'.format(ros_ip))
            os.environ['ROS_IP'] = ros_ip

            def ros_callback(msg):
                global ros_additional_message
                ros_additional_message = msg.data

            def ros_image_callback(msg):
                global ros_display_image
                bridge = cv_bridge.CvBridge()
                ros_display_image = bridge.imgmsg_to_cv2(
                    msg, desired_encoding='bgr8')

            rospy.init_node('atom_s3_display_information_node', anonymous=True)
            rospy.Subscriber('/atom_s3_additional_info', String, ros_callback,
                             queue_size=1)
            ros_available = True
            rate = rospy.Rate(1)
            sub = None
            while not rospy.is_shutdown() and not stop_event.is_set():
                ros_display_image_param = rospy.get_param(
                    '/display_image', None)
                if prev_ros_display_image_param != ros_display_image_param:
                    ros_display_image_flag = False
                    if sub is not None:
                        sub.unregister()
                        sub = None
                    if ros_display_image_param:
                        rospy.loginfo('Start subscribe {} for display'
                                      .format(ros_display_image_param))
                        ros_display_image_flag = True
                        sub = rospy.Subscriber(ros_display_image_param,
                                               sensor_msgs.msg.Image,
                                               queue_size=1,
                                               callback=ros_image_callback)
                prev_ros_display_image_param = ros_display_image_param
                rate.sleep()
            if rospy.is_shutdown():
                break
        except ImportError as e:
            print("ROS is not available ({}). Retrying...".format(e))
            time.sleep(5)  # Wait before retrying
        except rospy.ROSInterruptException as e:
            print("ROS interrupted ({}). Retrying...".format(e))
            time.sleep(5)  # Wait before retrying
        finally:
            ros_available = False
            ros_additional_message = None


def get_ip_address():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
    except socket.error as e:
        print(e)
    ip_address = s.getsockname()[0]
    s.close()
    if ip_address == '0.0.0.0':
        return None
    return ip_address


def get_ros_master_ip():
    master_str = os.getenv('ROS_MASTER_URI', default="None")
    # https://[IP Address]:11311 -> [IP Address]
    master_str = master_str.split(':')
    if len(master_str) > 1:
        master_ip = master_str[1].replace('/', '')
    else:
        return "none"
    return master_ip


def send_pisugar_command(command_str):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.connect(('localhost', 8423))
        s.sendall(command_str.encode())
        time.sleep(0.1)  # Wait for pisugar to response
        pisugar_str = s.recv(1024).decode()
    except Exception as e:
        print('{}: {}'.format(type(e), e))
    finally:
        s.close()
    return pisugar_str


def get_battery():
    battery_str = send_pisugar_command('get battery')
    try:
        # battery: [Battery Level]\n -> [Battery Level]
        tmp = battery_str.split(':')
        if len(tmp) > 1:
            battery_str = tmp[1].replace('\n', '')
            battery_level = int(float(battery_str))
        else:
            battery_level = None
    except Exception as e:
        print('{}: {}'.format(type(e), e))
        print('get battery result: {}'.format(battery_str))
        battery_level = None
    return(battery_level)


def battery_charging():
    charging_str = send_pisugar_command('get battery_charging')
    try:
        # battery_charging: [true/false]\n -> True/False
        if 'true' in charging_str:
            charging = True
        elif 'false' in charging_str:
            charging = False
        else:
            charging = None
    except Exception as e:
        print('{}: {}'.format(type(e), e))
        print('get battery_charging result: {}'.format(charging_str))
        charging = None
    return(charging)


class DisplayInformation(object):

    def __init__(self, i2c_addr):
        self.i2c_addr = i2c_addr
        self.i2c = busio.I2C(board.SCL1, board.SDA1)
        self.lock = FileLock(lock_path, timeout=10)

    def display_image(self, img):
        img = squared_padding_image(img, 128)
        quality = 75
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        result, jpg_img = cv2.imencode('.jpg', img, encode_param)
        jpg_size = len(jpg_img)

        header = []
        header += [0xFF, 0xD8, 0xEA]
        header += [(jpg_size & 0xFF00) >> 8,
                   (jpg_size & 0x00FF) >> 0]

        packer = WirePacker(buffer_size=1000)
        for h in header:
            packer.write(h)
        packer.end()

        if packer.available():
            self.i2c_write(packer.buffer[:packer.available()])

        time.sleep(0.005)

        for pack in nsplit(jpg_img, n=50):
            packer.reset()
            for h in [0xFF, 0xD8, 0xEA]:
                packer.write(h)
            for h in pack:
                packer.write(h)
            packer.end()
            if packer.available():
                self.i2c_write(packer.buffer[:packer.available()])
            time.sleep(0.005)

    def display_information(self):
        global ros_available
        global ros_additional_message

        ip = get_ip_address()
        if ip is None:
            ip = 'no connection'
        ip_str = '{}:\n{}{}{}'.format(
            socket.gethostname(), Fore.YELLOW, ip, Fore.RESET)
        master_str = 'ROS_MASTER:\n' + Fore.RED + '{}'.format(
            get_ros_master_ip()) + Fore.RESET
        battery = get_battery()
        if battery is None:
            battery_str = 'Bat: None'
        else:
            if battery <= 20:
                battery_str = 'Bat: {}{}%{}'.format(
                    Fore.RED, battery, Fore.RESET)
            else:
                battery_str = 'Bat: {}{}%{}'.format(
                    Fore.GREEN, battery, Fore.RESET)
        charging = battery_charging()
        if charging is True:
            battery_str += '+'
        elif charging is False:
            battery_str += '-'
        else:
            battery_str += '?'
        sent_str = '{}\n{}\n{}\n'.format(
            ip_str, master_str, battery_str)

        if ros_available and ros_additional_message:
            sent_str += '{}\n'.format(ros_additional_message)
            ros_additional_message = None

        print('send the following message')
        print(sent_str)
        packer = WirePacker(buffer_size=len(sent_str) + 8)
        for s in sent_str:
            packer.write(ord(s))
        packer.end()
        if packer.available():
            self.i2c_write(packer.buffer[:packer.available()])

    def display_qrcode(self, target_url=None):
        header = [0x02]
        if target_url is None:
            ip = get_ip_address()
            if ip is None:
                print('Could not get ip. skip showing qr code.')
                return
            target_url = 'http://{}:8085/riberry_startup/'.format(ip)
        header += [len(target_url)]
        header += list(map(ord, target_url))
        packer = WirePacker(buffer_size=100)
        for h in header:
            packer.write(h)
        packer.end()
        if packer.available():
            self.i2c_write(packer.buffer[:packer.available()])

    def i2c_write(self, packet):
        try:
            self.lock.acquire()
        except Timeout as e:
            print(e)
            return
        try:
            self.i2c.writeto(self.i2c_addr, packet)
        except OSError as e:
            print(e)
        except TimeoutError as e:
            print('I2C Write error {}'.format(e))
        try:
            self.lock.release()
        except Timeout as e:
            print(e)
            return

    def run(self):
        global ros_display_image
        global ros_display_image_flag

        while not stop_event.is_set():
            if ros_display_image_flag and ros_display_image is not None:
                self.display_image(ros_display_image)
            else:
                if get_ip_address() is None:
                    self.display_qrcode(
                        f'WIFI:S:radxa-{get_mac_address()};T:nopass;;')
                    time.sleep(3)
                else:
                    self.display_information()
                    time.sleep(3)
                    self.display_qrcode()
                    time.sleep(3)


if __name__ == '__main__':
    display_thread = threading.Thread(target=DisplayInformation(0x42).run)
    display_thread.daemon = True
    display_thread.start()

    try:
        try_init_ros()
    except KeyboardInterrupt:
        print("Interrupted by user, shutting down...")
        stop_event.set()
        display_thread.join()
