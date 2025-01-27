#!/usr/bin/env python3

import threading
import time

from riberry.com.uart_base import UARTBase
from riberry.com.uart_base import usb_devices
from riberry.network import get_ros_ip


def print_throttle(interval, *args, **kwargs):
    current_time = time.time()
    if not hasattr(print_throttle, 'last_print_time'):
        print_throttle.last_print_time = 0
    if current_time - print_throttle.last_print_time >= interval:
        print(*args, **kwargs)
        print_throttle.last_print_time = current_time



def find_pairing_devices(usb_ports, baudrate=115200):
    ret = []
    ports = usb_ports
    if len(ports) == 0:
        print("Cannot find USB devices")
        return ret
    device_types = ["Sender", "Receiver"]
    for port in ports:
        try:
            com = UARTBase(port, baudrate)
            print(f"Connect to USB device: {port}")
            while True:
                packet = com.read()
                if packet == b'':
                    print_throttle(1.0, "Waiting for initial packet from pairing device...")
                else:
                    device_type = packet.decode().strip()
                    if device_type in device_types:
                        print(f"[{device_type}] Pairing device is connected")
                        ret.append({'serial': com, 'device_type': device_type})
                    else:
                        print(f"Unknown device type: {device_type}")
                    break
                time.sleep(0.1)
        except Exception as e:
            print(f"Error: {e}")
            return ret
    return ret

def send_string(com, string):
    """Send IP address to ESP32"""
    try:
        com.write(string)
    except Exception as e:
        print(f"Send error: {e}")

def send_pairing_info(com, ip_address):
    send_string(com, ip_address + "\n")

def receive_pairing_info(com):
    try:
        while True:
            packet = com.read()
            if packet == b'':
                print_throttle(1.0, "[Receiver] Waiting for pairing information from Receiver...")
            else:
                pairing_info = packet.decode().strip()
                print(f"[Receiver] Receive pairing info: {pairing_info}")
                return pairing_info
            time.sleep(0.1)
    except Exception as e:
        print(f"Error: {e}")

def pairing(ports):
    pairing_devices = find_pairing_devices(ports)
    ip_address = get_ros_ip()
    for dev in pairing_devices:
        port = dev['serial'].serial.port
        if port in prev_ports:
            continue
        com = dev['serial']
        device_type = dev['device_type']
        # Use threading to test both Sender and Receiver device
        # on single computer
        if device_type == "Sender":
            print(f"[{device_type}] Send pairing info: {ip_address}")
            thread = threading.Thread(
                name=device_type,
                target=send_pairing_info,
                args=(com, ip_address,),
                daemon=True)
            thread.start()
        elif device_type == "Receiver":
            send_pairing_info(com, ip_address)  # This ip_address is not used
            print(f"[{device_type}] Start receive_pairing_info")
            thread = threading.Thread(
                name=device_type,
                target=receive_pairing_info,
                args=(com,),
                daemon=True)
            thread.start()


if __name__ == "__main__":
    # Find pairing device
    prev_ports = []
    while True:
        ports = usb_devices()
        new_ports = []
        for port in ports:
            if port not in prev_ports:
                new_ports.append(port)
        if len(new_ports) == 0:
            # Pairing is only once
            # print('No new USB device found')
            pass
        else:
            pairing(new_ports)
        prev_ports = ports
        time.sleep(0.1)
