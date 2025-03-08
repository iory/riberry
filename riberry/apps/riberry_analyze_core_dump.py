#!/usr/bin/env python3

import argparse

from riberry.com.base import ComBase
from riberry.com.i2c_base import I2CBase
from riberry.com.uart_base import UARTBase
from riberry.platformio.core_dump import read_core_dump


def main():
    parser = argparse.ArgumentParser(description='Read core dump from device.')
    parser.add_argument('--elf-path', '-e', type=str, default="",
                        help='Path to the ELF file')
    args = parser.parse_args()
    print(args.elf_path)

    device = ComBase.identify_device()
    if device in ['m5stack-LLM', 'Linux', 'Darwin']:
        com = UARTBase()
    else:
        com = I2CBase(0x42)

    read_core_dump(com, elf_path=args.elf_path)

if __name__ == "__main__":
    main()
