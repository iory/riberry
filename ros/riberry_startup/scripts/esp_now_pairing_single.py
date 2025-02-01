#!/usr/bin/env python3

import threading
import time

from riberry.com.uart_base import usb_devices
from riberry.esp_now_pairing import ESPNowPairing
from riberry.network import get_ip_address

"""
This program is for testing esp_now_pairing firmware
This program handles multiple esp_now_pairing devices by using threading.
"""

if __name__ == "__main__":
    esp_now_pairing = ESPNowPairing()
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
                pairing_devices = esp_now_pairing.find_USB_pairing_devices(new_ports)
                esp_now_pairing.set_pairing_info(get_ip_address())
                for dev in pairing_devices:
                    if dev is not None:
                        # Use threading to test both Sender and Receiver device
                        # on single computer
                        thread = threading.Thread(
                                        name=dev["device_type"],
                                        target=esp_now_pairing.pairing,
                                        args=(dev,),
                                        daemon=True)
                        thread.start()
            prev_ports = ports
        # The bus number and address of the I2C communication partner are known
        # so we can directly connect to the device
        elif com_type == "I2C":
            pairing_devices = esp_now_pairing.find_I2C_pairing_devices(0x42, 3)
            esp_now_pairing.pairing(pairing_devices[0])
        time.sleep(0.1)
