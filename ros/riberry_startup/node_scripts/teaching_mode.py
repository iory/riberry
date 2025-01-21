#!/usr/bin/env python3

from datetime import datetime
from enum import Enum
import os
import threading

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String

from riberry.com.base import PacketType
from riberry.com.i2c_base import I2CBase
from riberry.filecheck_utils import get_cache_dir
from riberry.select_list import SelectList
from riberry.teaching_manager import TeachingManager


class State(Enum):
    """
    Enum representing the states of the TeachingMode.

    Attributes:
        WAIT (int): Idle state where no action is being performed.
        RECORD (int): State for recording robot motions.
        PLAY (int): State for playing back recorded motions.
    """
    WAIT = 0
    RECORD = 1
    PLAY = 2


class TeachingMode(I2CBase):
    """
    TeachingMode is a class that manages robot motion teaching and playback through AtomS3 communication.

    This class provides functionality to:
    - Record robot motion and special actions. Then save them as JSON files via TeachingManager class
    - Play back recorded motions via TeachingManager class
    - Control these operations via AtomS3 button inputs

    Attributes:
        teaching_manager (TeachingManager): Handles motion and marker recording/playback. Actual motion control and marker recognition are performed by this instance.
        json_dir (str): Directory to save and load JSON files for recorded motions.
        play_list (SelectList): List of available motion files for playback.
        recording (bool): Indicates whether the system is currently recording motions.
        playing (bool): Indicates whether the system is currently playing motions.
        mode (str): Current operational mode, as set by the `/atom_s3_mode` topic.
        special_actions (list): Configurable list of special actions for recording.
        special_action_selected (dict): The currently selected special action during recording.
        special_action_executed (bool): Indicates if the selected special action is currently active.
        prev_state (State): The previous state of the system.
        state (State): The current state of the system.
        additional_str (str): Additional message string for display during operations.
    """

    def __init__(self, i2c_addr):
        super().__init__(i2c_addr)
        self.teaching_manager = TeachingManager()
        self.json_dir = get_cache_dir()
        self.play_list = SelectList()
        self.play_list.set_extract_pattern(r"teaching_(.*?)\.json")
        self.load_teaching_files()
        self.recording = False
        self.playing = False
        self.mode = None
        # Button and mode callback
        rospy.Subscriber(
            "/atom_s3_button_state",
            Int32, callback=self.button_cb, queue_size=1)
        rospy.Subscriber(
            "/atom_s3_mode", String, callback=self.mode_cb, queue_size=1)
        self.special_actions = rospy.get_param(
            '~special_actions', [])
        self.special_action_list = SelectList()
        for action in self.special_actions:
            keys = ("name","start_command", "stop_command")
            if not all(k in action for k in keys):
                rospy.logerr(f"special action must have key: {keys}.")
                return
            action["start_command"] = action["start_command"].replace(
                'ri.', 'self.ri.')
            action["stop_command"] = action["stop_command"].replace(
                'ri.', 'self.ri.')
            self.special_action_list.add_option(action["name"], position="end")
        self.special_action_selected = None
        self.special_action_executed = False
        self.prev_state = None
        self.state = State.WAIT
        rospy.Timer(rospy.Duration(0.1), self.update_atoms3)
        self.additional_str = ""

    def mode_cb(self, msg):
        """Callback to update the current operational mode.

        Args:
            msg (std_msgs.msg.String): Message containing the mode string.
        """
        if self.mode != "TeachingMode" and msg.data == "TeachingMode":
            rospy.loginfo("Start teaching mode")
        self.mode = msg.data

    def button_cb(self, msg):
        """Handles button state transitions for recording and playback.

        Args:
            msg (std_msgs.msg.Int32): Message containing the button state.

        Note:
            This class manages the internal state of its operations (e.g., self.state, self.playing, etc.).
            It does not directly manage or influence the UI updates or external displays, such as those handled by
            the self.update_atoms3 function.

            Basic state transitions:
            - WAIT -> RECORD -> WAIT (via single-click).
            - WAIT -> PLAY -> WAIT (via double-click).
        """
        if self.mode != "TeachingMode":
            return
        if self.prev_state != self.state:
            sent_str = f'State: {self.state.name}'
            rospy.loginfo(sent_str)
        self.prev_state = self.state
        # State transition
        if self.state == State.WAIT:
            self.handle_wait_state(msg)
        elif self.state == State.RECORD:
            self.handle_record_state(msg)
        elif self.state == State.PLAY:
            self.handle_play_state(msg)

    def handle_wait_state(self, msg):
        if msg.data == 1:
            self.state = State.RECORD
        elif msg.data == 2:
            self.state = State.PLAY
        elif msg.data == 3:
            self.teaching_manager.servo_off()

    def handle_record_state(self, msg):
        # select special action
        if self.special_action_not_selected():
            if msg.data == 1:
                self.special_action_list.increment_index()
            if msg.data == 2:
                self.special_action_selected = self.special_actions[self.special_action_list.get_index()]
            return
        # record until stopped
        if self.recording is False:
            self.start_recording()
        # Record and execute special action
        if msg.data == 1:
            if self.special_action_executed is True:
                action_state = 'Stop'
                command = self.special_action_selected["stop_command"]
            else:
                action_state = 'Start'
                command = self.special_action_selected["start_command"]
            # Record
            self.teaching_manager.motion_manager.add_action(
                self.teaching_manager.start_time,
                f'{action_state} {self.special_action_selected["name"]}',
                command)
            # Execute
            thread1 = threading.Thread(
                target=self.teaching_manager.motion_manager.exec_with_error_handling,
                args=(command,),
                daemon=True)
            thread1.start()
            self.special_action_executed = not self.special_action_executed
        # finish recording
        elif msg.data == 2:
            self.stop_recording()
        elif msg.data == 3:
            self.teaching_manager.servo_off()

    def handle_play_state(self, msg):
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
                self.start_playing()
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
                self.stop_playing()

    def special_action_not_selected(self):
        return len(self.special_actions) > 0 and\
            self.special_action_selected is None

    def update_atoms3(self, event):
        """Timer callback to update AtomS3 LCD with the current state and marker information.

        Args:
            event (rospy.TimerEvent): Timer event information.

        Note:
            This method is responsible for updating the UI and display information on the AtomS3 device.
            It does not modify the internal state of this class (e.g., self.state, self.playing, etc.).
            Instead, it reads these states and reflects them on the AtomS3 LCD.
        """
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
            sent_str += 'Record mode\n\n'
            # Select special action if ~special_actions param is defined
            if self.special_action_not_selected():
                sent_str += ' 1tap: next\n' + ' 2tap: select\n\n'
                sent_str += self.special_action_list.string_options(5)
            # Start recording
            else:
                sent_str += '1tap: '
                if self.special_action_selected is None:
                    sent_str += 'None\n\n'
                else:
                    sent_str += 'Action\n'
                    if self.special_action_executed is True:
                        sent_str += '\x1b[32m ON   \x1b[39m'
                    else:
                        sent_str += '\x1b[31m OFF  \x1b[39m'
                    sent_str += f'{self.special_action_selected["name"]}\n\n'
                sent_str += '2tap: finish\n\n'
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
        self.write(sent_str)

    def load_teaching_files(self):
        """Loads all teaching JSON files from the configured directory."""
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
        """Generates a path for a new or existing teaching JSON file.

        Args:
            filename (str, optional): Filename for the JSON file. Defaults to None.

        Returns:
            str: Full path to the teaching JSON file.
        """
        if filename is None:
            current_time = datetime.now().strftime("%m%d_%H%M%S")
            json_path = os.path.join(
                self.json_dir, f'teaching_{current_time}.json')
        else:
            json_path = os.path.join(
                self.json_dir, f'teaching_{filename}.json')
        return json_path

    def start_recording(self):
        """
        Initiates recording in a separate thread.
        """
        if self.recording:
            rospy.logwarn("Recording is already in progress")
            return
        self.recording = True
        self.special_action_executed = False

        def record_task():
            json_path = self.get_json_path()
            result_message = self.teaching_manager.record(json_path)
            self.additional_str = result_message
            self.play_list.add_option(json_path)

        self.record_thread = threading.Thread(
            target=record_task, daemon=True)
        self.record_thread.start()

    def stop_recording(self):
        self.teaching_manager.stop()
        self.state = State.WAIT
        self.recording = False
        self.special_action_selected = None

    def start_playing(self):
        """
        Initiates playback in a separate thread.
        """
        if self.playing:
            rospy.logwarn("Playback is already in progress")
            return
        self.playing = True

        def play_task():
            result_message = self.teaching_manager.play(
                self.play_file)
            self.additional_str = result_message
            # Automatically stop after play
            self.stop_playing()

        self.play_thread = threading.Thread(target=play_task, daemon=True)
        self.play_thread.start()

    def stop_playing(self):
        self.teaching_manager.stop()
        self.playing = False
        self.state = State.WAIT


if __name__ == '__main__':
    # Main
    rospy.init_node('teaching_mode', anonymous=True)
    TeachingMode(0x42)
    rospy.spin()
