#!/usr/bin/env python3

import argparse

from riberry.com.base import ComBase
from riberry.com.i2c_base import I2CBase
from riberry.com.uart_base import UARTBase
from riberry.platformio.core_dump import analyze_core_dump
from riberry.platformio.core_dump import read_core_dump


def main():
    parser = argparse.ArgumentParser(description='Read and analyze core dump from device.')
    parser.add_argument('--elf-path', '-e', type=str, default=None,
                        help='Path to the ELF file')
    parser.add_argument('--text', '-t', type=str, default=None,
                        help='Text to analyze core dump')
    parser.add_argument('--file', '-f', type=str, default=None,
                        help='File containing core dump text')
    args = parser.parse_args()

    if args.file:
        with open(args.file) as f:
            text = f.read()
        analyze_core_dump(text)
        return
    if args.text:
        analyze_core_dump(args.text)
        return
    device = ComBase.identify_device()
    if device in ['m5stack-LLM', 'Linux', 'Darwin']:
        com = UARTBase()
    else:
        com = I2CBase(0x42)

    read_core_dump(com, elf_path=args.elf_path)

if __name__ == "__main__":
    main()
