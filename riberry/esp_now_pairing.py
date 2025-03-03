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

# com is both USB and I2C
def get_role(com):
    while True:
        # Clear buffer to avoid mixed receive data like 'MainSecond'
        com.reset_input_buffer()
        com.write([PacketType.GET_PAIRING_TYPE])
        time.sleep(0.1)
        packet = com.read()
        if packet == b'':
            print_throttle(1.0, "Waiting for initial packet from pairing device...")
        else:
            if packet is None:
                print("Failed to get packet")
                return None
            role_string = packet.decode().strip()
            if is_valid_role(role_string):
                role = Role(role_string)
                print(f"[{role.value}] Pairing device is connected")
                return role
            else:
                print(f"Unknown device type: {role_string}")
                return None
            break
        time.sleep(0.1)
    return None

def find_USB_pairing_devices(usb_ports, baudrate=115200):
    ret = []
    ports = usb_ports
    if len(ports) == 0:
        print("Cannot find USB devices")
        return ret
    for port in ports:
        try:
            com = UARTBase(port, baudrate)
            print(f"Connect to USB device: {port}")
            role = get_role(com)
            if role is not None:
                ret.append({"com": com, "role": role})
        except Exception as e:
            print(f"Error: {e}")
            return ret
    return ret

def find_I2C_pairing_devices(device, bus):
    ret = []
    try:
        com = I2CBase(device, bus)
        print(f"Connect to I2C device: {device}")
        role = get_role(com)
        if role is not None:
            ret.append({"com": com, "role": role})
    except Exception as e:
        print(f"Error: {e}")
        return ret
    return ret

def pairing_info_to_packet(pairing_info):
    packet = [PacketType.SET_IP_REQUEST] + list(map(int, pairing_info.split('.')))
    return packet

def packet_to_pairing_info(packet):
    pairing_info = '.'.join(map(str, list(packet)))
    return pairing_info


class ESPNowPairing:
    def __init__(self, com, role):
        self.com = com
        self.role = role
        self.pairing_info = None  # e.g. "8.8.8.8"

    def send_string(self, string):
        """Send IP address to ESP32"""
        try:
            self.com.write(string)
        except Exception as e:
            print(f"Send error: {e}")

    def set_pairing_info(self, pairing_info):
        self.pairing_info = pairing_info

    def get_pairing_info(self):
        return self.pairing_info

    def send_pairing_info(self):
        packet = pairing_info_to_packet(self.pairing_info)
        self.send_string(packet)

    def receive_pairing_info(self, timeout=5):
        try:
            start_time = time.time()
            while True:
                if time.time() - start_time > timeout:
                    print(f"[{Role.Secondary.value}] Pairing timeout after {timeout} seconds.")
                    return False
                self.com.reset_input_buffer()
                self.com.write([PacketType.PAIRING_IP_REQUEST])
                time.sleep(0.1)
                packet = self.com.read()
                if packet == b'':
                    print_throttle(1.0, f"[{Role.Secondary.value}] Waiting for pairing info ...")
                elif len(packet) == 4 and packet != b'\xFF\xFF\xFF\xFF':
                    pairing_info = packet_to_pairing_info(packet)
                    self.pairing_info = pairing_info
                    print(f"[{Role.Secondary.value}] Receive pairing info: {pairing_info}")
                    return True
                time.sleep(0.1)
        except Exception as e:
            print(f"Error: {e}")

    def pairing(self):
        print(f'[{self.role.value}] Pairing start.')
        if self.role == Role.Main:
            print(f"[{self.role.value}] Send pairing info: {self.pairing_info}")
            self.send_pairing_info()
        elif self.role == Role.Secondary:
            print(f"[{self.role.value}] Start receive_pairing_info")
            self.receive_pairing_info()
