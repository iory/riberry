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


def get_teaching_files(json_dir):
    """Loads all teaching JSON files from the configured directory."""
    teaching_files = [
        os.path.join(json_dir, file)
        for file in os.listdir(json_dir)
        if file.startswith("teaching_") and file.endswith(".json")
    ]
    teaching_files_sorted = sorted(
        teaching_files,
        key=lambda f: os.path.getctime(f)
    )
    return teaching_files_sorted


def get_json_path(json_dir, filename=None):
    """Generates a path for a new or existing teaching JSON file.

    Args:
        filename (str, optional): Filename for the JSON file. Defaults to None.

    Returns:
        str: Full path to the teaching JSON file.
    """
    if filename is None:
        current_time = datetime.now().strftime("%m%d_%H%M%S")
        json_path = os.path.join(
            json_dir, f'teaching_{current_time}.json')
    else:
        json_path = os.path.join(
            json_dir, f'teaching_{filename}.json')
    return json_path


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


class AtomS3TeachingInterface(I2CBase):
    """
    Communicate with AtomS3 to manage current Mode and State.

    Attributes:
        prev_state (State): The previous state of the system.
        state (State): The current state of the system.
        mode (str): Current operational mode, as set by the `/atom_s3_mode` topic.
    """

    def __init__(self, i2c_addr):
        super().__init__(i2c_addr)
        self.mode = None
        rospy.Subscriber(
            "/atom_s3_mode", String, callback=self.mode_cb, queue_size=1)
        self.prev_state = None
        self.state = State.WAIT

    def mode_cb(self, msg):
        """Callback to update the current operational mode.

        Args:
            msg (std_msgs.msg.String): Message containing the mode string.
        """
        if self.mode != "TeachingMode" and msg.data == "TeachingMode":
            rospy.loginfo("Start teaching mode")
        self.mode = msg.data

    def print_state(self):
        if self.mode != "TeachingMode":
            return
        if self.prev_state != self.state:
            sent_str = f'State: {self.state.name}'
            rospy.loginfo(sent_str)
        self.prev_state = self.state

    def send_packet(self, sent_str):
        header = chr(PacketType.TEACHING_MODE)
        self.write(header+sent_str)


class TeachingMode:
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
        special_actions (list): Configurable list of special actions for recording.
        special_action_selected (dict): The currently selected special action during recording.
        special_action_executed (bool): Indicates if the selected special action is currently active.
        additional_str (str): Additional message string for display during operations.
    """

    def __init__(self, i2c_addr):
        self.atoms3_interface = AtomS3TeachingInterface(i2c_addr)
        self.teaching_manager = TeachingManager()
        self.recording = False
        self.prev_playing = False
        self.playing = False
        self.speed = rospy.get_param('~speed', 1.0)

        # Load teaching files
        self.play_list = SelectList()
        self.play_list.set_extract_pattern(r"teaching_(.*?)\.json")
        self.json_dir = get_cache_dir()
        for file in get_teaching_files(self.json_dir):
            self.play_list.add_option(file)

        # ROS callbacks
        rospy.Subscriber(
            "/atom_s3_button_state",
            Int32, callback=self.state_transition_cb, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self.update_atoms3)
        self.additional_str = ""

        # Special actions
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

    def state_transition_cb(self, msg):
        """Handles button state transitions for recording and playback.

        Args:
            msg (std_msgs.msg.Int32): Message containing the button state.

        Note:
            This class manages the internal state of its operations (e.g., self.playing, etc.).
            It does not directly manage or influence the UI updates or external displays, such as those handled by
            the self.update_atoms3 function.
        """

        # NOTE: To unify the management of self.atoms3_interface.state,
        # this variable must be changed
        # only within the state_transition_cb() method.
        if self.atoms3_interface.state == State.WAIT:
            self.atoms3_interface.state = self.handle_wait_state(msg)
        elif self.atoms3_interface.state == State.RECORD:
            self.atoms3_interface.state = self.handle_record_state(msg)
        elif self.atoms3_interface.state == State.PLAY:
            self.atoms3_interface.state = self.handle_play_state(msg)
        self.atoms3_interface.print_state()

    def handle_wait_state(self, msg: Int32) -> State:
        if msg.data == 1:
            return State.RECORD
        elif msg.data == 2:
            return State.PLAY
        elif msg.data == 3:
            self.teaching_manager.servo_off()
            return State.WAIT
        # If no button is pressed, stay State.WAIT and wait for button input
        return State.WAIT

    def handle_record_state(self, msg: Int32) -> State:
        # select special action
        if self.special_action_not_selected():
            if msg.data == 1:
                self.special_action_list.increment_index()
            if msg.data == 2:
                self.special_action_selected = self.special_actions[self.special_action_list.get_index()]
            return State.RECORD
        # record until stopped
        if self.recording is False:
            self.start_recording()
            return State.RECORD
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
            return State.RECORD
        # finish recording
        elif msg.data == 2:
            self.stop_recording()
            return State.WAIT
        elif msg.data == 3:
            self.teaching_manager.servo_off()
            return State.RECORD
        # If no button is pressed, stay State.RECORD and wait for button input
        return State.RECORD

    def handle_play_state(self, msg: Int32) -> State:
        next_state = None
        if self.prev_playing is True and self.playing is False:
            next_state = State.WAIT
        elif self.playing is False:
            if msg.data != 0 and len(self.play_list.options) <= 0:
                next_state = State.WAIT
            # select play file
            elif msg.data == 1:
                self.play_list.increment_index()
                next_state = State.PLAY
            # confirm play file
            elif msg.data == 2:
                self.play_file = self.play_list.selected_option()
                self.start_playing()
                next_state = State.PLAY
            # delete play file
            elif msg.data == 3:
                delete_file = self.play_list.selected_option()
                if os.path.exists(delete_file):
                    os.remove(delete_file)
                self.play_list.remove_option(delete_file)
                next_state = State.WAIT
        else:
            # stop playing
            if msg.data == 2:
                self.stop_playing()
                next_state = State.WAIT

        self.prev_playing = self.playing
        # If no button is pressed, stay State.PLAY and wait for button input
        if next_state is None:
            next_state = State.PLAY
        return next_state

    def update_atoms3(self, event):
        """Timer callback to update AtomS3 LCD with the current state and marker information.

        Args:
            event (rospy.TimerEvent): Timer event information.

        Note:
            This method is responsible for updating the UI and display information on the AtomS3 device.
            It does not modify the internal state of this class (e.g., self.playing, etc.).
            Instead, it reads these states and reflects them on the AtomS3 LCD.
        """
        if self.atoms3_interface.mode != "TeachingMode":
            # When mode is changed,
            # state automatically returns to WAIT
            self.atoms3_interface.state = State.WAIT
            self.play_list.reset_index()
            return

        sent_str = ""
        if self.teaching_manager.marker_manager.is_marker_recognized():
            marker_ids = self.teaching_manager.marker_manager.current_marker_ids()
            sent_str += str(marker_ids[0])
        delimiter = ','
        sent_str += delimiter
        if self.atoms3_interface.state == State.WAIT:
            sent_str += 'Teaching mode\n\n'\
                + '1tap: record\n'\
                + '2tap: play\n'\
                + '3tap: free'
            if self.additional_str != "":
                sent_str += "\n\n" + self.additional_str
        elif self.atoms3_interface.state == State.RECORD:
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
        elif self.atoms3_interface.state == State.PLAY:
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
                sent_str += f'\n\nSpeed x{self.speed}'
        delimiter_num = 1
        if len([char for char in sent_str if char == delimiter]) != delimiter_num:
            rospy.logerr(f"sent string: {sent_str}")
            rospy.logerr(f"The number of delimiter '{delimiter}' must be {delimiter_num}")
        self.atoms3_interface.send_packet(sent_str)

    def special_action_not_selected(self):
        return len(self.special_actions) > 0 and\
            self.special_action_selected is None

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
            json_path = get_json_path(self.json_dir)
            result_message = self.teaching_manager.record(json_path)
            self.additional_str = result_message
            self.play_list.add_option(json_path)

        self.record_thread = threading.Thread(
            target=record_task, daemon=True)
        self.record_thread.start()

    def stop_recording(self):
        self.teaching_manager.stop()
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
                self.play_file, self.speed)
            self.additional_str = result_message
            # Automatically stop after play
            self.stop_playing()

        self.play_thread = threading.Thread(target=play_task, daemon=True)
        self.play_thread.start()

    def stop_playing(self):
        self.teaching_manager.stop()
        self.playing = False


if __name__ == '__main__':
    # Main
    rospy.init_node('teaching_mode', anonymous=True)
    TeachingMode(0x42)
    rospy.spin()
