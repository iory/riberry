#!/usr/bin/env python3

import time

import board
import busio
from filelock import FileLock
from filelock import Timeout

from i2c_for_esp32 import WirePacker
import rospy
from speech_recognition_msgs.msg import SpeechRecognitionCandidates


def i2c_write(lock, i2c, i2c_addr, packet):
    try:
        lock.acquire()
    except Timeout as e:
        print(e)
        return
    try:
        i2c.writeto(i2c_addr, packet)
    except OSError as e:
        print(e)
    except TimeoutError as e:
        print('I2C Write error {}'.format(e))
    try:
        lock.release()
    except Timeout as e:
        print(e)
        return


if __name__ == '__main__':
    rospy.init_node('speak_to_light')

    i2c = busio.I2C(board.SCL3, board.SDA3)
    i2c_addr = 0x41
    while not i2c.try_lock():
        pass

    lock_path = '/tmp/i2c-3.lock'
    lock = FileLock(lock_path, timeout=10)

    def callback(msg):
        print(msg.transcript[0])
        i2c.unlock()
        while not i2c.try_lock():
            pass

        input_string = chr(255) + chr(0) + chr(0)
        packer = WirePacker(buffer_size=len(input_string) + 8)
        for s in input_string:
            packer.write(ord(s))
        packer.end()
        if packer.available():
            i2c_write(lock, i2c, i2c_addr, packer.buffer[:packer.available()])
        rospy.sleep(2.0)

        input_string = chr(255) + chr(255) + chr(255)
        packer = WirePacker(buffer_size=len(input_string) + 8)
        for s in input_string:
            packer.write(ord(s))
        packer.end()
        if packer.available():
            i2c_write(lock, i2c, i2c_addr, packer.buffer[:packer.available()])
        rospy.sleep(2.0)


    sub = rospy.Subscriber('speech_to_text', SpeechRecognitionCandidates,
                           queue_size=1,
                           callback=callback)
    rospy.spin()
