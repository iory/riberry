#!/usr/bin/env python3

import os
import socket
import subprocess
import sys
import time
import threading
import io
import fcntl
from collections import Counter

import cv2
from colorama import Fore
from i2c_for_esp32 import WirePacker
from pybsc.image_utils import squared_padding_image
from pybsc import nsplit
from filelock import FileLock
from filelock import Timeout
import smbus2

IP5209_CURVE = [
    (4160, 100),
    (4050, 95),
    (4000, 80),
    (3920, 65),
    (3860, 40),
    (3790, 25),
    (3660, 10),
    (3520, 6),
    (3490, 3),
    (3100, 0),
]

I2C_SLAVE = 0x0703

if sys.hexversion < 0x03000000:
    def _b(x):
        return x
else:
    def _b(x):
        return x.encode('latin-1')


class i2c:

    def __init__(self, device=0x42, bus=5):
        self.fr = io.open("/dev/i2c-"+str(bus), "rb", buffering=0)
        self.fw = io.open("/dev/i2c-"+str(bus), "wb", buffering=0)
        # set device address
        fcntl.ioctl(self.fr, I2C_SLAVE, device)
        fcntl.ioctl(self.fw, I2C_SLAVE, device)

    def write(self, data):
        if type(data) is list:
            data = bytearray(data)
        elif type(data) is str:
            data = _b(data)
        self.fw.write(data)

    def read(self, count):
        return self.fr.read(count)

    def close(self):
        self.fw.close()
        self.fr.close()


# Ensure that the standard output is line-buffered. This makes sure that
# each line of output is flushed immediately, which is useful for logging.
# This is for systemd.
sys.stdout.reconfigure(line_buffering=True)


pisugar_battery_percentage = None
debug_battery = False
debug_i2c_text = False


def identify_device():
    try:
        with open('/proc/cpuinfo', 'r') as f:
            cpuinfo = f.read()

        if 'Raspberry Pi' in cpuinfo:
            return 'Raspberry Pi'

        with open('/proc/device-tree/model', 'r') as f:
            model = f.read().strip()

        # remove null character
        model = model.replace('\x00', '')

        if 'Radxa' in model or 'ROCK Pi' in model or model in 'Khadas VIM4':
            return model

        return 'Unknown Device'
    except FileNotFoundError:
        return 'Unknown Device'


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
    global pisugar_battery_percentage
    ros_display_image_param = None
    prev_ros_display_image_param = None
    while not stop_event.is_set():
        try:
            import rospy
            from std_msgs.msg import String
            from std_msgs.msg import Float32
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
            battery_pub = rospy.Publisher('/pisugar_battery', Float32,
                                          queue_size=1)
            ros_available = True
            rate = rospy.Rate(1)
            sub = None
            while not rospy.is_shutdown() and not stop_event.is_set():
                ros_display_image_param = rospy.get_param(
                    '/display_image', None)
                if pisugar_battery_percentage is not None:
                    battery_pub.publish(pisugar_battery_percentage)
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


class MP2760BatteryMonitor(object):

    BATTERY_DEVICE_ADDRESS = 0x5c
    REG27H = 0x27  # Battery Charge Current
    REG26H = 0x26  # System Voltage
    REG24H = 0x24  # Input Current
    REG25H = 0x25  # Battery Voltage

    def __init__(self, bus_number=3):
        self.bus = smbus2.SMBus(bus_number)

    def set_adc_continuous_mode(self, set_bit=True):
        try:
            word = self.bus.read_word_data(
                self.BATTERY_DEVICE_ADDRESS,
                0x0e)
        except Exception as e:
            print('[Battery Monitor] Error reading from I2C: {}'.format(e))
            return
        current_bit = (word >> 7) & 1
        if current_bit == set_bit:
            print(
               '[Battery Monitor] 7bit is already set to the desired value. ',
               'No action needed.')
            return
        if set_bit:
            print('[Battery Monitor] Set ADC_CONV to Continuous')
            set_adc_word = word | (1 << 7)
        else:
            print('[Battery Monitor] Set ADC_CONV to One-shot conversion')
            set_adc_word = word & ~(1 << 7)
        try:
            self.bus.write_word_data(
                self.BATTERY_DEVICE_ADDRESS,
                0x0e, set_adc_word)
        except Exception as e:
            print('[Battery Monitor] Error writing I2C: {}'.format(e))
            return

    def calculate_lipo_percentage(self, voltage):
        max_voltage = 8.4  # 100%
        min_voltage = 6.0  # 0%

        if voltage >= max_voltage:
            return 100
        elif voltage <= min_voltage:
            return 0
        percentage = (voltage - min_voltage) / (
           max_voltage - min_voltage) * 100
        return round(percentage, 2)

    def read_register(self, register):
        try:
            value = self.bus.read_word_data(
               self.BATTERY_DEVICE_ADDRESS, register)
        except Exception as e:
            print('[Battery Monitor] {}'.format(e))
            return None
        return value

    def calculate_voltage_from_bits(self, bits):
        voltage = 0
        voltage += ((bits & 0x200) >> 9) * 2560
        voltage += ((bits & 0x100) >> 8) * 1280
        voltage += ((bits & 0x080) >> 7) * 640
        voltage += ((bits & 0x040) >> 6) * 320
        voltage += ((bits & 0x020) >> 5) * 160
        voltage += ((bits & 0x010) >> 4) * 80
        voltage += ((bits & 0x008) >> 3) * 40
        voltage += ((bits & 0x004) >> 2) * 20
        voltage += ((bits & 0x002) >> 1) * 10
        voltage += (bits & 0x001) * 5
        return voltage

    def read_charge_status(self):
        """Read charge status

0: No charging
1: Trickle charge
2: Pre-charge
3: CC charge
4: CV charge
5: Charge termination
"""
        reg16_value = self.read_register(0x16)
        if reg16_value is None:
            return None
        charge_status = (reg16_value & 0x1c0) >> 6
        return charge_status

    def read_input_voltage(self):
        reg23_value = self.read_register(0x23)
        if reg23_value is None:
            return None
        input_voltage = 0
        input_voltage += ((reg23_value & 0x200) >> 9) * 10240
        input_voltage += ((reg23_value & 0x100) >> 8) * 5120
        input_voltage += ((reg23_value & 0x080) >> 7) * 2560
        input_voltage += ((reg23_value & 0x040) >> 6) * 1280
        input_voltage += ((reg23_value & 0x020) >> 5) * 640
        input_voltage += ((reg23_value & 0x010) >> 4) * 320
        input_voltage += ((reg23_value & 0x008) >> 3) * 160
        input_voltage += ((reg23_value & 0x004) >> 2) * 80
        input_voltage += ((reg23_value & 0x002) >> 1) * 40
        input_voltage += (reg23_value & 0x001) * 20
        return 0.001 * input_voltage

    def read_system_voltage(self):
        reg26_value = self.read_register(self.REG26H)
        if reg26_value is None:
            return None
        system_voltage = 0
        system_voltage += ((reg26_value & 0x200) >> 9) * 10240
        system_voltage += ((reg26_value & 0x100) >> 8) * 5120
        system_voltage += ((reg26_value & 0x080) >> 7) * 2560
        system_voltage += ((reg26_value & 0x040) >> 6) * 1280
        system_voltage += ((reg26_value & 0x020) >> 5) * 640
        system_voltage += ((reg26_value & 0x010) >> 4) * 320
        system_voltage += ((reg26_value & 0x008) >> 3) * 160
        system_voltage += ((reg26_value & 0x004) >> 2) * 80
        system_voltage += ((reg26_value & 0x002) >> 1) * 40
        system_voltage += (reg26_value & 0x001) * 20
        return 0.001 * system_voltage

    def read_battery_voltage(self):
        reg25_value = self.read_register(self.REG25H)
        if reg25_value is None:
            return None
        battery_voltage = 2 * self.calculate_voltage_from_bits(reg25_value)
        return 0.001 * battery_voltage

    def read_input_current(self, register_address):
        reg24_value = self.read_register(register_address)
        if reg24_value is None:
            return None
        input_current = 0
        input_current += ((reg24_value & 0x200) >> 9) * 3200
        input_current += ((reg24_value & 0x100) >> 8) * 1600
        input_current += ((reg24_value & 0x080) >> 7) * 800
        input_current += ((reg24_value & 0x040) >> 6) * 400
        input_current += ((reg24_value & 0x020) >> 5) * 200
        input_current += ((reg24_value & 0x010) >> 4) * 100
        input_current += ((reg24_value & 0x008) >> 3) * 50
        input_current += ((reg24_value & 0x004) >> 2) * 25
        input_current += ((reg24_value & 0x002) >> 1) * 12.5
        input_current += (reg24_value & 0x001) * 6.25
        return input_current

    def read_junction_temperature(self):
        reg2a_value = self.read_register(0x2A)
        if reg2a_value is None:
            return None
        junction_temp = 0
        junction_temp += ((reg2a_value & 0x200) >> 9) * 512
        junction_temp += ((reg2a_value & 0x100) >> 8) * 256
        junction_temp += ((reg2a_value & 0x080) >> 7) * 128
        junction_temp += ((reg2a_value & 0x040) >> 6) * 64
        junction_temp += ((reg2a_value & 0x020) >> 5) * 32
        junction_temp += ((reg2a_value & 0x010) >> 4) * 16
        junction_temp += ((reg2a_value & 0x008) >> 3) * 8
        junction_temp += ((reg2a_value & 0x004) >> 2) * 4
        junction_temp += ((reg2a_value & 0x002) >> 1) * 2
        junction_temp += (reg2a_value & 0x001) * 1
        return 314 - 0.5703 * junction_temp

    def get_filtered_percentage(self):
        self.set_adc_continuous_mode(set_bit=True)
        battery_voltage = self.read_battery_voltage()
        if battery_voltage is None:
            return None
        return self.calculate_lipo_percentage(battery_voltage)

    def get_is_charging(self):
        charge_status = self.read_charge_status()
        self.set_adc_continuous_mode(set_bit=False)
        return charge_status != 0


def majority_vote(history):
    if not history:
        return 0
    count = Counter(history)
    return count.most_common(1)[0][0]


class PisugarBatteryReader(threading.Thread):

    def __init__(self, bus_number=1, device_address=0x57, alpha=0.9,
                 value_threshold=1000, percentage_threshold=20,
                 history_size=10):
        super().__init__()
        self.bus_number = bus_number
        self.device_address = device_address
        self.alpha = alpha
        self.percentage_threshold = percentage_threshold
        self.history_size = history_size

        self.filtered_percentage = 0
        self.percentage_history = []
        self.charging_history = []

        self.bus = smbus2.SMBus(self.bus_number)
        self.lock = threading.Lock()
        self.running = True

    def read_sensor_data(self, get_charge=False):
        try:
            if get_charge is True:
                value = self.bus.read_byte_data(
                    self.device_address, 0x02) >> 7
            else:
                value = self.bus.read_byte_data(self.device_address, 0x2A)
            return value
        except OSError:
            # for pisugar2
            try:
                vol_low = self.bus.read_byte_data(0x75, 0xa2)
                vol_high = self.bus.read_byte_data(0x75, 0xa3)
                if not (0 <= vol_low <= 255 and 0 <= vol_high <= 255):
                    print("[Pisugar Battery Reader] Invalid voltage data")
                    return
                if (vol_high & 0x20) == 0x20:
                    vol = 2600 - (~vol_low + (~(vol_high & 0x1F)) * 256 + 1)\
                       * 27 // 100
                else:
                    vol = 2600 + (vol_low + vol_high * 256) * 27 // 100
                cap = 0
                for i in range(len(IP5209_CURVE)):
                    if vol >= IP5209_CURVE[i][0]:
                        cap = IP5209_CURVE[i][1]
                        if i == 0:
                            break
                    if i > 0:
                        vol_diff_v = vol - IP5209_CURVE[i][0]
                        k_percent = IP5209_CURVE[i - 1][1] - IP5209_CURVE[i][1]
                        k_voltage = IP5209_CURVE[i - 1][0] - IP5209_CURVE[i][0]
                        k = k_percent / k_voltage
                        cap += int(k * vol_diff_v)
                        break
                return cap
            except Exception as e:
                print('[Pisugar Battery Reader] {}'.format(e))
                return None
        except Exception as e:
            print('[Pisugar Battery Reader] {}'.format(e))
            return None

    def is_outlier(self, current, history, threshold):
        if not history:
            return False
        ratio = sum(abs(current - h) > threshold
                    for h in history) / len(history)
        return ratio > 0.4

    def update_history(self, value, history):
        history.append(value)
        if len(history) > self.history_size:
            history.pop(0)

    def run(self):
        try:
            while self.running:
                percentage = self.read_sensor_data()
                is_charging = self.read_sensor_data(get_charge=True)
                if percentage is None or is_charging is None:
                    time.sleep(0.2)
                    continue

                with self.lock:
                    if self.is_outlier(
                          percentage,
                          self.percentage_history,
                          self.percentage_threshold):
                        pass
                        print(f"Percentage outlier detected:",
                              f" {percentage:.2f}, ",
                              f"history: {self.percentage_history}")
                    else:
                        self.filtered_percentage = \
                           (self.alpha * percentage +
                            (1 - self.alpha) * self.filtered_percentage)
                    # Always update history
                    self.update_history(percentage, self.percentage_history)

                    self.charging_history.append(is_charging)
                    if len(self.charging_history) > self.history_size:
                        self.charging_history.pop(0)

                if debug_battery:
                    print(f"RAW Percentage: {percentage:.2f}")
                    print(f"Filtered Percentage:",
                          f" {self.filtered_percentage:.2f}")
                time.sleep(0.2)
        finally:
            self.bus.close()

    def get_filtered_percentage(self):
        with self.lock:
            return self.filtered_percentage

    def get_is_charging(self):
        with self.lock:
            return majority_vote(self.charging_history) == 1

    def stop(self):
        self.running = False
        self.join()


class DisplayInformation(object):

    def __init__(self, i2c_addr):
        self.i2c_addr = i2c_addr
        self.device_type = identify_device()
        if self.device_type == 'Raspberry Pi':
            import board
            import busio
            self.i2c = busio.I2C(board.SCL, board.SDA)
            bus_number = 1
        elif self.device_type == 'Radxa Zero':
            import board
            import busio
            self.i2c = busio.I2C(board.SCL1, board.SDA1)
            bus_number = 3
        elif self.device_type == 'Khadas VIM4':
            self.i2c = i2c()
            bus_number = None
        else:
            raise ValueError('Unknown device {}'.format(
                self.device_type))
        self.lock = FileLock(lock_path, timeout=10)
        use_pisugar = False
        try:
            battery_monitor = MP2760BatteryMonitor(bus_number)
            voltage = battery_monitor.read_battery_voltage()
            if voltage is None:
                print('[Display Information] Use Pisugar')
                use_pisugar = True
            else:
                print('[Display Information] Use JSK Battery Board')
        except Exception:
            print('[Display Information] Use JSK Battery Board')
        self.use_pisugar = use_pisugar
        if bus_number:
            if use_pisugar:
                self.battery_reader = PisugarBatteryReader(bus_number)
                self.battery_reader.daemon = True
                self.battery_reader.start()
            else:
                self.battery_reader = MP2760BatteryMonitor(bus_number)
        else:
            self.battery_reader = None

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
        global pisugar_battery_percentage

        ip = get_ip_address()
        if ip is None:
            ip = 'no connection'
        ip_str = '{}:\n{}{}{}'.format(
            socket.gethostname(), Fore.YELLOW, ip, Fore.RESET)
        master_str = 'ROS_MASTER:\n' + Fore.RED + '{}'.format(
            get_ros_master_ip()) + Fore.RESET
        battery_str = ''
        if self.battery_reader:
            battery = self.battery_reader.get_filtered_percentage()
            pisugar_battery_percentage = battery
            if battery is None:
                battery_str = 'Bat: None'
            else:
                if battery <= 20:
                    battery_str = 'Bat: {}{}%{}'.format(
                        Fore.RED, int(battery), Fore.RESET)
                else:
                    battery_str = 'Bat: {}{}%{}'.format(
                        Fore.GREEN, int(battery), Fore.RESET)
            # charging = battery_charging()
            charging = self.battery_reader.get_is_charging()
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

        if debug_i2c_text:
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
                    if self.device_type == 'Raspberry Pi':
                        ssid = f'raspi-{get_mac_address()}'
                    elif self.device_type == 'Radxa Zero':
                        ssid = f'radxa-{get_mac_address()}'
                    else:
                        ssid = f'radxa-{get_mac_address()}'
                    self.display_qrcode(f'WIFI:S:{ssid};T:nopass;;')
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
