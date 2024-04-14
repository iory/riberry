#!/usr/bin/env python

import os
import socket
import time

import board
import busio
from colorama import Fore
from i2c_for_esp32 import WirePacker


def get_ip_address():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    for _ in range(6):
        try:
            s.connect(("8.8.8.8", 80))
        except socket.error as e:
            print(e)
            print('Try to connect network again')
            time.sleep(10)
        else:
            break
    ip_address = s.getsockname()[0]
    s.close()
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
        self.i2c = busio.I2C(board.SCL1, board.SDA1, frequency=400_000)
        while not self.i2c.try_lock():
            pass

    def display_information(self):
        ip_str = '{}:\n{}{}{}'.format(
            socket.gethostname(), Fore.YELLOW, get_ip_address(), Fore.RESET)
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
        print('send the following message')
        print(sent_str)
        packer = WirePacker(buffer_size=len(sent_str) + 8)
        for s in sent_str:
            packer.write(ord(s))
        packer.end()
        if packer.available():
            self.i2c.writeto(self.i2c_addr,
                             packer.buffer[:packer.available()])

    def run(self):
        while True:
            self.display_information()
            time.sleep(10)


if __name__ == '__main__':
    act = DisplayInformation(0x42)
    act.run()
