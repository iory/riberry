#!/usr/bin/env python3

import argparse

from riberry.com.base import ComBase
from riberry.com.i2c_base import I2CBase
from riberry.com.uart_base import UARTBase
from riberry.firmware_update import update_firmware


def main():
    parser = argparse.ArgumentParser(description='Update firmware for the device.')
    parser.add_argument('--firmware-path', '-f', required=True, help='Path to the firmware file')
    args = parser.parse_args()

    device = ComBase.identify_device()
    if device in ['m5stack-LLM', 'Linux', 'Darwin']:
        com = UARTBase()
    else:
        com = I2CBase(0x42)
    update_firmware(com, firmware_path=args.firmware_path)


if __name__ == "__main__":
    main()
