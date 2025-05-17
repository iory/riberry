#!/usr/bin/env python3

import rospy
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from std_msgs.msg import String

from riberry.chatbot import Chatbot


class ChatbotNode:
    def __init__(self):
        system_prompt = rospy.get_param(
            '~system_prompt', 'You are a helpful assistant.')
        model = "gpt-4.1-nano"
        self.chatbot = Chatbot(system_prompt, model)
        rospy.loginfo(f"ChatbotNode - model: {model}")
        rospy.loginfo(f"System prompt: {system_prompt}")
        rospy.Subscriber('~input', SpeechRecognitionCandidates, self.cb)
        self.pub = rospy.Publisher('~output', String, queue_size=1)

    def cb(self, msg):
        user_prompt = msg.transcript[0]
        rospy.loginfo(f"Received user prompt: {user_prompt}")
        system_output = self.chatbot.generate_response(user_prompt)
        rospy.loginfo(f"Generated response: {system_output}")
        self.pub.publish(system_output)

if __name__ == '__main__':
    rospy.init_node('chatbot_node')
    node = ChatbotNode()
    rospy.spin()
