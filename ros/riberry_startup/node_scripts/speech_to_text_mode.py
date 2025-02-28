#!/usr/bin/env python

from colorama import Back
import rospy
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from std_msgs.msg import Int32

from riberry.com.base import PacketType
from riberry.mode import Mode


class SpeechToTextMode(Mode):
    def __init__(self):
        super().__init__()
        self.pub = rospy.Publisher('speech_to_text', SpeechRecognitionCandidates, queue_size=1)
        rospy.Subscriber('speech_to_text_raw', SpeechRecognitionCandidates, self.speech_cb)
        rospy.Subscriber("atom_s3_button_state", Int32, callback=self.button_cb, queue_size=1)
        self.passthrough = False
        self.text = ""
        self.prev_mode = None

    def mode_cb(self, msg):
        self.prev_mode = self.mode
        super().mode_cb(msg)
        if self.prev_mode != self.mode:
            self.update_lcd()

    def button_cb(self, msg):
        if self.mode != "SpeechToTextMode":
            return
        if msg.data != 1:
            return
        self.passthrough = not self.passthrough
        if self.passthrough is True:
            rospy.loginfo("Start passthrough speech_to_text topic")
        else:
            rospy.loginfo("Stop passthrough speech_to_text topic")
        self.update_lcd()

    def speech_cb(self, msg):
        if self.mode != "SpeechToTextMode":
            return
        self.text = msg.transcript[0]
        if self.passthrough is True:
            self.pub.publish(msg)
        self.update_lcd()

    def update_lcd(self):
        sent_str = chr(PacketType.SPEECH_TO_TEXT_MODE)
        if self.passthrough is True:
            sent_str += f"1tap: {Back.GREEN}Active{Back.RESET} Idle\n\n"
        else:
            sent_str += f"1tap: Active {Back.RED}Idle{Back.RESET}\n\n"
        sent_str += "You said\n"
        sent_str += f"{self.text}\n"
        self.write(sent_str)


if __name__ == '__main__':
    rospy.init_node('speech_to_text_mode')
    SpeechToTextMode()
    rospy.spin()
