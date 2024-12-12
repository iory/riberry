#!/usr/bin/env python3

from datetime import datetime
from enum import Enum
import os

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String

from riberry.i2c_base import I2CBase
from riberry.i2c_base import PacketType
from riberry.motion_manager import MotionManager
from riberry.select_list import SelectList


class State(Enum):
    WAIT = 0
    RECORD = 1
    PLAY = 2


class TeachingMode(I2CBase):
    def __init__(self, i2c_addr):
        super().__init__(i2c_addr)
        self.motion_manager = MotionManager()
        self.json_dir = os.path.join(os.environ["HOME"], ".ros/riberry")
        os.makedirs(self.json_dir, exist_ok=True)
        self.play_list = SelectList()
        self.play_list.set_extract_pattern(r"teaching_(.*?)\.json")
        self.load_teaching_files()
        self.playing = False
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
Wait -> (Double-click) -> Play -> (Double-click) -> Confirm -> (Double-click) -> Abort -> Wait
"""
        if self.mode != "TeachingMode":
            return
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
            elif msg.data == 3:
                self.motion_manager.servo_off()
        elif self.state == State.RECORD:
            # finish recording
            if msg.data == 1:
                self.motion_manager.stop()
                self.state = State.WAIT
            elif msg.data == 3:
                self.motion_manager.servo_off()
        elif self.state == State.PLAY:
            if self.playing is False:
                if msg.data != 0 and len(self.play_list.options) <= 0:
                    self.state = State.WAIT
                    return
                # select play file
                if msg.data == 1:
                    self.play_list.increment_index()
                # confirm play file
                elif msg.data == 2:
                    self.play_file = self.play_list.selected_option()
                    self.playing = True
                # delete play file
                elif msg.data == 3:
                    delete_file = self.play_list.selected_option()
                    if os.path.exists(delete_file):
                        os.remove(delete_file)
                    self.play_list.remove_option(delete_file)
                    self.state = State.WAIT
            else:
                # stop playing
                if msg.data == 2:
                    self.motion_manager.stop()
                    self.state = State.WAIT

    def timer_callback(self, event):
        if self.mode != "TeachingMode":
            # When mode is changed,
            # state automatically returns to WAIT
            self.state = State.WAIT
            self.play_list.reset_index()
            return
        sent_str = chr(PacketType.TEACHING_MODE)
        marker_msg = self.motion_manager.marker_msg
        if marker_msg is not None and len(marker_msg.detections) > 0:
            marker_id = marker_msg.detections[0].id[0]
            sent_str += str(marker_id)
        delimiter = ','
        sent_str += delimiter
        if self.state == State.WAIT:
            sent_str += 'Teaching mode\n\n'\
                + '1tap:\n record\n\n'\
                + '2tap:\n play\n\n'\
                + '3tap:\n servo_off'
        elif self.state == State.RECORD:
            sent_str += 'Record mode\n\n'\
                + '1tap: finish'
        elif self.state == State.PLAY:
            if self.playing is False:
                if len(self.play_list.options) <= 0:
                    sent_str += 'Play mode\n\n'\
                        + 'No motion\n'\
                        + ' 1 tap: return'
                else:
                    sent_str += 'Play mode\n'\
                        + ' 1tap: select\n'\
                        + ' 2tap: start\n'\
                        + ' 3tap: delete\n\n'
                    sent_str += self.play_list.string_options(5)
            else:
                sent_str += 'Play mode\n\n'\
                    + f'{self.play_list.selected_option(True)}\n\n'\
                    + '2tap:\n stop playing'
        delimiter_num = 1
        if len([char for char in sent_str if char == delimiter]) != delimiter_num:
            print(f"The number of delimiter '{delimiter}' "
                  f"must be {delimiter_num}")
        self.send_string(sent_str)

    def load_teaching_files(self):
        teaching_files = [
            os.path.join(self.json_dir, file)
            for file in os.listdir(self.json_dir)
            if file.startswith("teaching_") and file.endswith(".json")
        ]
        teaching_files_sorted = sorted(
            teaching_files,
            key=lambda f: os.path.getctime(f)
        )
        for file in teaching_files_sorted:
            self.play_list.add_option(file)

    def get_json_path(self, filename=None):
        if filename is None:
            current_time = datetime.now().strftime("%m%d_%H%M%S")
            json_path = os.path.join(
                self.json_dir, f'teaching_{current_time}.json')
        else:
            json_path = os.path.join(
                self.json_dir, f'teaching_{filename}.json')
        return json_path

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
                    # record until stopped
                    json_path = self.get_json_path()
                    self.motion_manager.record(json_path)
                    self.play_list.add_option(json_path)
                    self.state = State.WAIT
                elif self.state == State.PLAY:
                    # wait for play file to be confirmed
                    if self.playing is False:
                        continue
                    else:
                        self.motion_manager.play(self.play_file)
                        self.playing = False
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
