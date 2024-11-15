#!/usr/bin/env python3

from enum import Enum

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String

from riberry.i2c_base import I2CBase
from riberry.motion_manager import MotionManager


class State(Enum):
    WAIT = 0
    RECORD = 1
    PLAY = 2

class TeachingMode(I2CBase):
    def __init__(self, i2c_addr, lock_path="/tmp/teaching_mode.lock"):
        super().__init__(i2c_addr)
        self.motion_manager = MotionManager()
        # Button and mode callback
        self.mode = None
        rospy.Subscriber(
            "/atom_s3_button_state",
            Int32, callback=self.button_cb, queue_size=1)
        rospy.Subscriber(
            "/atom_s3_mode", String, callback=self.mode_cb, queue_size=1)
        self.prev_state = None
        self.state = State.WAIT
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def mode_cb(self, msg):
        self.mode = msg.data

    def button_cb(self, msg):
        """
        Manage state by click
Wait -> (Single-click) -> Record  -> (Single-click) -> Finish recording -> Wait
Wait -> (Double-click) -> Play -> (Double-click) -> Abort -> Wait
"""
        if self.mode != "TeachingMode":
            return
        if msg.data == 3:
            self.motion_manager.servo_off()
        if self.prev_state != self.state:
            sent_str = f'State: {self.state.name}'
            rospy.loginfo(sent_str)
        self.prev_state = self.state
        # State transition
        if self.state == State.WAIT:
            if msg.data == 1:
                self.state = State.RECORD
            elif msg.data == 2:
                self.state = State.PLAY
        elif self.state == State.RECORD:
            if msg.data == 1:
                self.motion_manager.stop()
                self.state = State.WAIT
        elif self.state == State.PLAY:
            if msg.data == 2:
                self.motion_manager.stop()
                self.state = State.WAIT

    def timer_callback(self, event):
        if self.mode != "TeachingMode":
            return
        sent_str = ''
        if self.state == State.WAIT:
            sent_str += 'Teaching mode\n\n'\
                + 'single click:\n  record\n\n'\
                + 'double click:\n  play\n\n'\
                + 'triple click:\n  servo_off'
        elif self.state == State.RECORD:
            sent_str += 'Record mode\n\n'\
                + 'single click:\n  stop recording'
        elif self.state == State.PLAY:
            sent_str += 'Play mode\n\n'\
                + 'double click:\n  stop playing'
        self.send_string(sent_str)

    def main_loop(self):
        rospy.loginfo('start teaching mode')
        try:
            while not rospy.is_shutdown():
                if self.mode != "TeachingMode":
                    rospy.sleep(1)
                    continue
                if self.state == State.WAIT:
                    rospy.sleep(1)
                elif self.state == State.RECORD:
                    self.motion_manager.record()
                    self.state = State.WAIT
                elif self.state == State.PLAY:
                    self.motion_manager.play()
                    self.state = State.WAIT
        except KeyboardInterrupt:
            print('Finish teaching mode by KeyboardInterrupt')
            exit()


if __name__ == '__main__':
    # Main
    rospy.init_node('teaching_mode', anonymous=True)
    tm = TeachingMode(0x42)
    # To stop program by Ctrl-c
    import signal
    signal.signal(signal.SIGINT, signal.default_int_handler)
    tm.main_loop()
