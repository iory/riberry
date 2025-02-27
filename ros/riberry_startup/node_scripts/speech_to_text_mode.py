#!/usr/bin/env python

from colorama import Back
import rospy
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from std_msgs.msg import Int32
from std_msgs.msg import String

from riberry.com.base import ComBase
from riberry.com.uart_base import UARTBase


class SpeechToTextMode:
    def __init__(self):
        device = ComBase.identify_device()
        if device != 'm5stack-LLM':
            return
        self.com = UARTBase()
        self.pub = rospy.Publisher('speech_to_text', SpeechRecognitionCandidates, queue_size=1)
        rospy.Subscriber('speech_to_text_raw', SpeechRecognitionCandidates, self.speech_cb)
        rospy.Subscriber("atom_s3_mode", String, callback=self.mode_cb, queue_size=1)
        rospy.Subscriber("atom_s3_button_state", Int32, callback=self.button_cb, queue_size=1)
        rospy.Timer(rospy.Duration(1), self.timer_cb)
        self.mode = None
        self.passthrough = False
        self.text = ""

    def button_cb(self, msg):
        if msg.data != 1:
            return
        if self.passthrough is True:
            rospy.loginfo("Stop passthrough speech_to_text topic")
        else:
            rospy.loginfo("Start passthrough speech_to_text topic")
        self.passthrough = not self.passthrough

    def mode_cb(self, msg):
        self.mode = msg.data

    def speech_cb(self, msg):
        if self.mode != "SpeechToTextMode":
            return
        self.text = msg.transcript[0]
        rospy.logerr("com write speech recognition")
        if self.passthrough is True:
            self.pub.publish(msg)

    def timer_cb(self, event):
        if self.mode != "SpeechToTextMode":
            return
        self.com.write("Speech Recognition\n")
        if self.passthrough is True:
            self.com.write(f"1tap: {Back.GREEN}ON{Back.RESET}\n\n")
        else:
            self.com.write(f"1tap: {Back.RED}OFF{Back.RESET}\n\n")
        self.com.write(f"{self.text}\n")


if __name__ == '__main__':
    rospy.init_node('speech_to_text_mode')
    SpeechToTextMode()
    rospy.spin()
