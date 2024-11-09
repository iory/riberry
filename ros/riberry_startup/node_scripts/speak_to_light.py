#!/usr/bin/env python3


from i2c_for_esp32 import WirePacker
import rospy
from speech_recognition_msgs.msg import SpeechRecognitionCandidates

from riberry.i2c_base import I2C

if __name__ == "__main__":
    rospy.init_node("speak_to_light")

    i2c = I2C(0x41, bus=3)

    def callback(msg):
        print(msg.transcript[0])
        input_string = chr(255) + chr(0) + chr(0)
        packer = WirePacker(buffer_size=len(input_string) + 8)
        for s in input_string:
            packer.write(ord(s))
        packer.end()
        if packer.available():
            i2c.write(packer.buffer[: packer.available()])
        rospy.sleep(2.0)

        input_string = chr(255) + chr(255) + chr(255)
        packer = WirePacker(buffer_size=len(input_string) + 8)
        for s in input_string:
            packer.write(ord(s))
        packer.end()
        if packer.available():
            i2c.write(packer.buffer[: packer.available()])
        rospy.sleep(2.0)

    sub = rospy.Subscriber(
        "speech_to_text", SpeechRecognitionCandidates, queue_size=1, callback=callback
    )
    rospy.spin()
