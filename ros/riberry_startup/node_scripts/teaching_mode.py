#!/usr/bin/env python3

from datetime import datetime
from enum import Enum
import os

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String

from riberry.i2c_base import I2CBase
from riberry.i2c_base import PacketType
from riberry.select_list import SelectList
from riberry.teaching_manager import TeachingManager


class State(Enum):
    WAIT = 0
    RECORD = 1
    PLAY = 2


class TeachingMode(I2CBase):
    """
    TeachingMode is a class that manages robot motion teaching and playback through AtomS3 communication.

    This class provides functionality to:
    1. Record robot motions and save them as JSON files via TeachingManager class
    2. Play back recorded motions via TeachingManager class
    3. Control these operations via AtomS3 button inputs

    Actual motion control and marker recognition are performed by a subordinate class, TeachingManager.
    """

    def __init__(self, i2c_addr):
        super().__init__(i2c_addr)
        self.teaching_manager = TeachingManager()
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
        self.special_action_name = rospy.get_param(
            '~special_action_name', None)
        self.special_action_start_command = rospy.get_param(
            '~special_action_start_command', None)
        self.special_action_stop_command = rospy.get_param(
            '~special_action_stop_command', None)
        self.prev_state = None
        self.state = State.WAIT
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        self.additional_str = ""

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
                self.teaching_manager.servo_off()
        elif self.state == State.RECORD:
            # finish recording
            if msg.data == 1:
                self.teaching_manager.stop()
                self.state = State.WAIT
            # Record special action
            elif msg.data == 2:
                self.teaching_manager.motion_manager.add_action(
                    self.teaching_manager.start_time,
                    self.special_action_name,
                    self.special_action_start_command,
                    self.special_action_stop_command)
            elif msg.data == 3:
                self.teaching_manager.servo_off()
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
                    self.teaching_manager.stop()
                    self.state = State.WAIT

    def timer_callback(self, event):
        if self.mode != "TeachingMode":
            # When mode is changed,
            # state automatically returns to WAIT
            self.state = State.WAIT
            self.play_list.reset_index()
            return
        sent_str = chr(PacketType.TEACHING_MODE)
        if self.teaching_manager.marker_manager.is_marker_recognized():
            marker_ids = self.teaching_manager.marker_manager.current_marker_ids()
            sent_str += str(marker_ids[0])
        delimiter = ','
        sent_str += delimiter
        if self.state == State.WAIT:
            sent_str += 'Teaching mode\n\n'\
                + '1tap: record\n'\
                + '2tap: play\n'\
                + '3tap: free'
            if self.additional_str != "":
                sent_str += "\n\n" + self.additional_str
        elif self.state == State.RECORD:
            self.additional_str = ""
            sent_str += 'Record mode\n\n'\
                + '1tap: finish\n'
            sent_str += '2tap: '
            if self.special_action_name is None or\
               self.special_action_start_command is None or\
               self.special_action_stop_command is None:
                sent_str += 'None\n'
            else:
                sent_str += f'{self.special_action_name}\n'
            sent_str += '3tap: free'
        elif self.state == State.PLAY:
            self.additional_str = ""
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
            rospy.logerr(f"sent string: {sent_str}")
            rospy.logerr(f"The number of delimiter '{delimiter}' must be {delimiter_num}")
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
                    result_message = self.teaching_manager.record(
                        json_path)
                    self.additional_str = result_message
                    self.play_list.add_option(json_path)
                    self.state = State.WAIT
                elif self.state == State.PLAY:
                    # wait for play file to be confirmed
                    if self.playing is False:
                        continue
                    else:
                        result_message = self.teaching_manager.play(
                            self.play_file)
                        self.additional_str = result_message
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
