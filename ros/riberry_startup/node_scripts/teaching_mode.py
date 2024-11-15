#!/usr/bin/env python3

from datetime import datetime
from enum import Enum
import os

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String

from riberry.i2c_base import I2CBase
from riberry.motion_manager import MotionManager
from riberry.select_list import SelectList


class State(Enum):
    WAIT = 0
    RECORD = 1
    PLAY = 2


class TeachingMode(I2CBase):
    def __init__(self, i2c_addr, lock_path="/tmp/teaching_mode.lock"):
        super().__init__(i2c_addr)
        self.motion_manager = MotionManager()
        self.ros_dir = os.path.join(os.environ["HOME"], ".ros")
        self.select_list = SelectList()
        self.select_list.add_extract_pattern(r"teaching_(.*?)\.json")
        self.add_teaching_files()
        self.motion_file = None
        self.mode = None
        # Button and mode callback
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
            if self.motion_file is None:
                if msg.data == 1:
                    self.select_list.increment_index()
                elif msg.data == 2:
                    self.motion_file = self.select_list.get_selected()
                    if self.motion_file is None:
                        self.state = State.WAIT
            else:
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
            if self.motion_file is None:
                sent_str += 'Play mode\n\n'\
                    + 'double click: start playing\n\n'
                sent_str += self.select_list.get_list_string(5)
            else:
                sent_str += 'Play mode\n\n'\
                    + f'{self.select_list.get_selected(True)}\n\n'\
                    + 'double click:\n  stop playing'
        self.send_string(sent_str)

    def add_teaching_files(self):
        teaching_files = [
            os.path.join(self.ros_dir, file) for file in os.listdir(self.ros_dir)
            if file.startswith("teaching_") and file.endswith(".json")
        ]

        teaching_files_sorted = sorted(
            teaching_files,
            key=lambda f: os.path.getctime(f)
        )
        for file in teaching_files_sorted:
            self.select_list.add_option(file)

    def get_json_filepath(self, filename=None):
        if filename is not None:
            json_filepath = os.path.join(self.ros_dir, f'teaching_{filename}.json')
        else:
            current_time = datetime.now().strftime("%m%d_%H%M%S")
            json_filepath = os.path.join(self.ros_dir, f'teaching_{current_time}.json')
        return json_filepath

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
                    motion_path = self.get_json_filepath()
                    self.motion_manager.record(motion_path)
                    self.select_list.add_option(motion_path)
                    self.state = State.WAIT
                elif self.state == State.PLAY:
                    if self.motion_file is None:
                        continue
                    else:
                        self.motion_manager.play(self.motion_file)
                        self.motion_file = None
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
