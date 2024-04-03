#!/usr/bin/env python3

import time

import board
import busio

from i2c_for_esp32 import WirePacker
import rospy
from speech_recognition_msgs.msg import SpeechRecognitionCandidates


if __name__ == '__main__':
    rospy.init_node('speak_to_light')

    i2c = busio.I2C(board.SCL3, board.SDA3, frequency=1_000_000)
    i2c_addr = 0x41
    while not i2c.try_lock():
        pass


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
            i2c.writeto(i2c_addr, packer.buffer[:packer.available()])
        rospy.sleep(2.0)

        input_string = chr(255) + chr(255) + chr(255)
        packer = WirePacker(buffer_size=len(input_string) + 8)
        for s in input_string:
            packer.write(ord(s))
        packer.end()
        if packer.available():
            i2c.writeto(i2c_addr, packer.buffer[:packer.available()])
        rospy.sleep(2.0)


    sub = rospy.Subscriber('speech_to_text', SpeechRecognitionCandidates,
                           queue_size=1,
                           callback=callback)
    rospy.spin()
