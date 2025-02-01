#!/usr/bin/env python3

import threading
import time

from riberry.com.uart_base import usb_devices
from riberry.esp_now_pairing import ESPNowPairing
from riberry.esp_now_pairing import find_USB_pairing_devices
from riberry.esp_now_pairing import Role
from riberry.network import get_ip_address

"""
This program is for testing esp_now_pairing firmware
This program handles multiple esp_now_pairing devices by using threading.
"""


def pairing_impl(com, role):
    esp_now_pairing = ESPNowPairing(com=com, role=role)
    if esp_now_pairing.role == Role.Main:
        esp_now_pairing.set_pairing_info(get_ip_address())
    esp_now_pairing.pairing()


if __name__ == "__main__":
    com_type = "USB"
    prev_ports = []
    while True:
        # USB will be unplugged and plugged in,
        # so sometimes the port will be present and sometimes not.
        # so we need to check if there is a new device
        if com_type == "USB":
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
                pairing_devices = find_USB_pairing_devices(new_ports)
                for dev in pairing_devices:
                    if dev is not None:
                        # Use threading to test both Sender and Receiver device
                        # on single computer
                        thread = threading.Thread(
                            name=dev["role"],
                            target=pairing_impl,
                            args=(dev["com"], dev["role"],),
                            daemon=True)
                        thread.start()
            prev_ports = ports
        time.sleep(0.1)
