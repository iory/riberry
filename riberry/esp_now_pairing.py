from enum import Enum
import time

from riberry.com.base import PacketType
from riberry.com.i2c_base import I2CBase
from riberry.com.uart_base import UARTBase


class Role(Enum):
    Main = "Main"
    Secondary = "Secondary"
# Define aliases
Role._value2member_map_["Second"] = Role.Secondary


def is_valid_role(role_string):
    return role_string in Role._value2member_map_


def print_throttle(interval, *args, **kwargs):
    current_time = time.time()
    if not hasattr(print_throttle, 'last_print_time'):
        print_throttle.last_print_time = 0
    if current_time - print_throttle.last_print_time >= interval:
        print(*args, **kwargs)
        print_throttle.last_print_time = current_time


class ESPNowPairing:
    def __init__(self):
        self.pairing_info = None  # e.g. "8.8.8.8"
        self.device_type = None

    # com is both USB and I2C
    def detect_device_type(self, com):
        while True:
            # Clear buffer to avoid mixed receive data like 'MainSecond'
            com.reset_input_buffer()
            com.write([PacketType.GET_PAIRING_TYPE])
            time.sleep(0.1)
            packet = com.read()
            if packet == b'':
                print_throttle(1.0, "Waiting for initial packet from pairing device...")
            else:
                role_string = packet.decode().strip()
                if is_valid_role(role_string):
                    device_type = Role(role_string)
                    print(f"[{device_type.value}] Pairing device is connected")
                    return {'com': com, 'device_type': device_type}
                else:
                    print(f"Unknown device type: {role_string}")
                break
            time.sleep(0.1)
        return None

    def find_USB_pairing_devices(self, usb_ports, baudrate=115200):
        ret = []
        ports = usb_ports
        if len(ports) == 0:
            print("Cannot find USB devices")
            return ret
        for port in ports:
            try:
                com = UARTBase(port, baudrate)
                print(f"Connect to USB device: {port}")
                dev = self.detect_device_type(com)
                if dev is not None:
                    ret.append(dev)
            except Exception as e:
                print(f"Error: {e}")
                return ret
        return ret

    def find_I2C_pairing_devices(self, device, bus):
        ret = []
        try:
            com = I2CBase(device, bus)
            print(f"Connect to I2C device: {device}")
            dev = self.detect_device_type(com)
            if dev is not None:
                ret = dev
        except Exception as e:
            print(f"Error: {e}")
            return ret
        return ret

    def send_string(self, com, string):
        """Send IP address to ESP32"""
        try:
            com.write(string)
        except Exception as e:
            print(f"Send error: {e}")

    def pairing_info_to_packet(self, pairing_info):
        packet = [PacketType.SET_IP_REQUEST] + list(map(int, self.pairing_info.split('.')))
        return packet

    def packet_to_pairing_info(self, packet):
        pairing_info = '.'.join(map(str, list(packet)))
        return pairing_info

    def set_pairing_info(self, pairing_info):
        self.pairing_info = pairing_info

    def get_pairing_info(self):
        return self.pairing_info

    def send_pairing_info(self, com):
        packet = self.pairing_info_to_packet(self.pairing_info)
        self.send_string(com, packet)

    def receive_pairing_info(self, com):
        try:
            while True:
                com.reset_input_buffer()
                com.write([PacketType.PAIRING_IP_REQUEST])
                time.sleep(0.1)
                packet = com.read()
                if packet == b'':
                    print_throttle(1.0, f"[{self.device_type.value}] Waiting for pairing info ...")
                elif len(packet) == 4:
                    pairing_info = self.packet_to_pairing_info(packet)
                    self.pairing_info = pairing_info
                    print(f"[{self.device_type.value}] Receive pairing info: {pairing_info}")
                    return
                time.sleep(0.1)
        except Exception as e:
            print(f"Error: {e}")

    def pairing(self, pairing_device):
        com = pairing_device['com']
        self.device_type = pairing_device['device_type']
        print(f'[{self.device_type.value}] Pairing start.')
        if self.device_type == Role.Main:
            print(f"[{self.device_type.value}] Send pairing info: {self.pairing_info}")
            self.send_pairing_info(com)
        elif self.device_type == Role.Secondary:
            print(f"[{self.device_type.value}] Start receive_pairing_info")
            self.receive_pairing_info(com)
