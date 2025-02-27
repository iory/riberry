#!/usr/bin/env python

from colorama import Back
import rospy
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from std_msgs.msg import Int32

from riberry.com.base import ComBase
from riberry.com.base import PacketType
from riberry.com.uart_base import UARTBase
from riberry.mode import Mode


class SpeechToTextMode(Mode):
    def __init__(self):
        super().__init__()
        device = ComBase.identify_device()
        if device != 'm5stack-LLM':
            return
        self.com = UARTBase()
        self.pub = rospy.Publisher('speech_to_text', SpeechRecognitionCandidates, queue_size=1)
        rospy.Subscriber('speech_to_text_raw', SpeechRecognitionCandidates, self.speech_cb)
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
        self.update_lcd()

    def speech_cb(self, msg):
        if self.mode != "SpeechToTextMode":
            return
        self.text = msg.transcript[0]
        rospy.logerr("com write speech recognition")
        if self.passthrough is True:
            self.pub.publish(msg)

    def update_lcd(self):
        sent_str = chr(PacketType.TEXT)
        if self.mode != "SpeechToTextMode":
            return
        if self.passthrough is True:
            sent_str += f"1tap: {Back.GREEN}Active{Back.RESET} Idle\n\n"
        else:
            sent_str += f"1tap: Active {Back.RED}Idle{Back.RESET}\n\n"
        sent_str += "You Said\n"
        sent_str += f"{self.text}\n"
        self.com.write(sent_str)

    def timer_cb(self, event):
        self.update_lcd()

if __name__ == '__main__':
    rospy.init_node('speech_to_text_mode')
    SpeechToTextMode()
    rospy.spin()
