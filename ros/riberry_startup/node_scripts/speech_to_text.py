#!/usr/bin/env python

# Mainly copied from
# https://github.com/jsk-ros-pkg/jsk_3rdparty/blob/master/respeaker_ros/scripts/speech_to_text.py


import rospy

try:
    import speech_recognition as SR
except ImportError as e:
    raise ImportError(str(e) + '\nplease try "pip install speechrecognition"')

from audio_common_msgs.msg import AudioData
from audio_common_msgs.msg import AudioInfo
from speech_recognition_msgs.msg import SpeechRecognitionCandidates


class SpeechToText:
    def __init__(self):
        # language of STT service
        self.language = rospy.get_param("~language", "ja-JP")

        # ignore voice input while the robot is speaking
        self.self_cancellation = rospy.get_param("~self_cancellation", True)
        self.recognizer = SR.Recognizer()
        self.is_canceling = False
        self.pub_speech = rospy.Publisher(
            "speech_to_text", SpeechRecognitionCandidates, queue_size=1)
        self.sub_audio = rospy.Subscriber("audio", AudioData, self.audio_cb)

        # Get audio info
        _audio_info = rospy.wait_for_message('audio_info', AudioInfo)
        self.sample_rate = _audio_info.sample_rate
        if _audio_info.sample_rate not in [8000, 16000, 32000, 48000]:
            rospy.logerr('sampling rate must be 8000 or 16000 or 32000 or 48000')
            return
        if _audio_info.sample_format == 'S16LE':
            self.sample_width = 2
        elif _audio_info.sample_format == 'S32LE':
            self.sample_width = 4
        else:
            rospy.logerr('audio format must be [S16LE, S32LE]')
            return

    def audio_cb(self, msg):
        if self.is_canceling:
            rospy.loginfo("Speech is cancelled")
            return

        data = SR.AudioData(msg.data, self.sample_rate, self.sample_width)
        try:
            rospy.loginfo(f"Waiting for result {len(data.get_raw_data())}")
            result = self.recognizer.recognize_google(
                data, language=self.language)
            msg = SpeechRecognitionCandidates(
                transcript=[result],
                confidence=[1.0],
            )
            self.pub_speech.publish(msg)
        except SR.UnknownValueError as e:
            rospy.logerr(f"Failed to recognize: {e!s}")
        except SR.RequestError as e:
            rospy.logerr(f"Failed to recognize: {e!s}")


if __name__ == '__main__':
    rospy.init_node("speech_to_text")
    stt = SpeechToText()
    rospy.spin()
