#!/usr/bin/env python

import rospy
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from std_msgs.msg import String


class DisplaySpeechToText:
    def __init__(self):
        self.pub = rospy.Publisher('atom_s3_additional_info', String, queue_size=1)
        rospy.Subscriber('speech_to_text', SpeechRecognitionCandidates, self.callback)

    def callback(self, msg):
        output_msg = String()
        output_msg.data = msg.transcript[0]
        self.pub.publish(output_msg)
        rospy.loginfo("Republished: %s", output_msg.data)


if __name__ == '__main__':
    rospy.init_node('display_speech_to_text')
    DisplaySpeechToText()
    rospy.spin()
