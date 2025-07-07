#!/usr/bin/env python

# Mainly copied from https://github.com/jsk-ros-pkg/jsk_3rdparty/blob/master/webrtcvad_ros/node_scripts/webrtcvad_ros.py

import sys

try:
    import webrtcvad
except ImportError as e:
    print('\x1b[31m' + str(e) + '\nplease try "pip install webrtcvad"' + '\x1b[39m')
    sys.exit(1)

from audio_common_msgs.msg import AudioData
from audio_common_msgs.msg import AudioInfo
import numpy as np
import rospy
from std_msgs.msg import Bool


class WebRTCVADROS:

    def __init__(self):

        self._current_speaking = False
        self._speech_audio_buffer = ''
        self._last_speech_time = None

        aggressiveness = rospy.get_param('~aggressiveness', 1)
        self._minimum_duration = rospy.get_param('~minimum_duration', 0.4)
        self._silence_wait_time = rospy.get_param('~silence_wait_time', 0.5)
        self._vad = webrtcvad.Vad(int(aggressiveness))

        self._pub_is_speech = rospy.Publisher('~is_speaking', Bool, queue_size=1)
        self._pub_speech_audio = rospy.Publisher('~speech_audio', AudioData, queue_size=1)
        self._pub_speech_audio_info = rospy.Publisher('~speech_audio_info', AudioInfo, queue_size=1, latch=True)

        self._audio_info = rospy.wait_for_message('audio_info', AudioInfo)
        if self._audio_info.sample_format not in ['S16LE', 'S32LE']:
            rospy.logerr('audio format must be [S16LE, S32LE]')
            return
        if self._audio_info.sample_rate not in [8000, 16000, 32000, 48000]:
            rospy.logerr('sampling rate must be 8000 or 16000 or 32000 or 48000')
            return

        self._pub_speech_audio_info.publish(self._audio_info)
        self._sub = rospy.Subscriber('audio_data', AudioData, self._callback)

    def _callback(self, msg):
        if self._audio_info.sample_format == 'S16LE':
            audio_data = np.frombuffer(msg.data, dtype=np.int16)
        elif self._audio_info.sample_format == 'S32LE':
            audio_data = np.array(np.frombuffer(msg.data, dtype=np.int32) >> 16,
                                  dtype=np.int16)
        is_speech = self._vad.is_speech(audio_data.tobytes(), self._audio_info.sample_rate)
        self._pub_is_speech.publish(Bool(is_speech))

        current_time = rospy.Time.now()

        if is_speech:
            if not self._current_speaking:
                self._speech_audio_buffer = msg.data
                self._current_speaking = True
            else:
                self._speech_audio_buffer += msg.data
            self._last_speech_time = current_time
        else:
            if self._current_speaking:
                if self._last_speech_time and (current_time - self._last_speech_time).to_sec() < self._silence_wait_time:
                    self._speech_audio_buffer += msg.data
                else:
                    speech_duration = len(self._speech_audio_buffer) / self._audio_info.sample_rate
                    if speech_duration > self._minimum_duration:
                        self._pub_speech_audio.publish(AudioData(self._speech_audio_buffer))
                    else:
                        rospy.logwarn(f'speech duration: {speech_duration} dropped')
                    self._current_speaking = False
                    self._speech_audio_buffer = ''


def main():

    rospy.init_node('webrtcvad_ros')
    WebRTCVADROS()
    rospy.spin()


if __name__ == '__main__':
    main()
