#!/usr/bin/env python3

from datetime import datetime
from enum import Enum
import os
import threading

from riberry_startup.srv import SelectMotion
from riberry_startup.srv import SelectMotionResponse
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_srvs.srv import SetBool
from std_srvs.srv import SetBoolResponse

from riberry.com.base import PacketType
from riberry.filecheck_utils import get_cache_dir
from riberry.mode import Mode
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
    Each State corresponds to each screen of AtomS3.

    Attributes:
        WAIT (int): Idle state where no action is being performed.
        SPECIAL_ACTION_SELECT (int): State for selecting special action (optional)
        RECORD (int): State for recording robot motions.
        SPECIAL_ACTION_SELECT (int): State for selecting file to play
        PLAY (int): State for playing back recorded motions.
    """
    WAIT = 0
    SPECIAL_ACTION_SELECT = 1
    RECORD = 2
    PLAY_LIST_SELECT = 3
    PLAY = 4
    MOTION_LIST_SELECT = 5
    CHANGE_MOTION_NAME = 6


class TeachingMode(Mode):
    """
    TeachingMode is a class that manages robot motion teaching and playback through AtomS3 communication.

    This class provides functionality to:
    - Record robot motion and special actions. Then save them as JSON files via TeachingManager class
    - Play back recorded motions via TeachingManager class
    - Control these operations via AtomS3 button inputs

    Attributes:
        teaching_manager (TeachingManager): Handles motion and marker recording/playback. Actual motion control and marker recognition are performed by this instance.
        mode (str): Current operational mode, as set by the `/atom_s3_mode` topic.
        prev_state (State): The previous state of the system.
        state (State): The current state of the system.
        json_dir (str): Directory to save and load JSON files for recorded motions.
        play_list (SelectList): List of available motion files for playback.
        playing (bool): Indicates whether the system is currently playing motions.
        special_actions (list): Configurable list of special actions for recording.
        special_action_selected (dict): The currently selected special action during recording.
        special_action_executed (bool): Indicates if the selected special action is currently active.
        additional_str (str): Additional message string for display during operations.
    """

    def __init__(self):
        super().__init__()
        self.init_finished = False
        rospy.Timer(rospy.Duration(0.1), self.update_atoms3)
        self.teaching_manager = TeachingManager()
        self.playing = False
        self.new_motion_name = None
        self.speed = rospy.get_param('~speed', 1.0)
        self.load_play_list()

        # ROS callbacks
        self.prev_state = State.WAIT
        self.state = State.WAIT
        self.additional_str = ""
        rospy.Subscriber(
            "atom_s3_button_state",
            Int32, callback=self.state_transition_cb, queue_size=1)
        rospy.Subscriber(
            "teaching_mode_additional_info",
            String, callback=self.additional_info_cb, queue_size=1)

        # Call action from rosservice
        rospy.Service('~play', SelectMotion, self.play_srv)
        rospy.Service('~record', SetBool, self.record_srv)
        rospy.Service('~special_action', SetBool, self.special_action_srv)
        rospy.Service('~change_name', SetBool, self.change_name_srv)
        self.virtual_button_pub = rospy.Publisher(
            "atom_s3_button_state", Int32, queue_size=1)

        # Special actions
        self.special_actions = rospy.get_param(
            '~special_actions', [])
        self.special_action_list = SelectList()
        for action in self.special_actions:
            keys = ("name", "start_command", "stop_command")
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

        self.init_finished = True

    def mode_cb(self, msg):
        """Callback to update the current operational mode.

        Args:
            msg (std_msgs.msg.String): Message containing the mode string.
        """
        if self.mode != "TeachingMode" and msg.data == "TeachingMode":
            rospy.loginfo("Start teaching mode")
        super().mode_cb(msg)

    def state_transition_cb(self, msg):
        """Handles button state transitions for recording and playback.

        Args:
            msg (std_msgs.msg.Int32): Message containing the button state.

        Note:
            This class manages the internal state of its operations (e.g., self.state, self.playing, etc.).
            It does not directly manage or influence the UI updates or external displays, such as those handled by
            the self.update_atoms3 function.
        """
        if self.mode != "TeachingMode":
            return
        # NOTE: To unify the management of self.state,
        # this variable must be changed
        # only within the state_transition_cb() method.
        if self.state == State.WAIT:
            self.state = self.handle_wait_state(msg)
        elif self.state == State.SPECIAL_ACTION_SELECT:
            self.state = self.handle_special_action_select_state(msg)
        elif self.state == State.RECORD:
            self.state = self.handle_record_state(msg)
        elif self.state == State.PLAY_LIST_SELECT:
            self.state = self.handle_play_list_select_state(msg)
        elif self.state == State.PLAY:
            self.state = self.handle_play_state(msg)
        elif self.state == State.MOTION_LIST_SELECT:
            self.state = self.handle_motion_list_select_state(msg)
        elif self.state == State.CHANGE_MOTION_NAME:
            self.state = self.handle_change_motion_name_state(msg)
        # Print state transition
        if self.prev_state != self.state:
            rospy.loginfo(f'State: {self.state.name}')
        self.prev_state = self.state

    def wait_for_state(self, target_state, timeout=None):
        if timeout is not None:
            start_time = rospy.Time.now()
            timeout_duration = rospy.Duration(timeout)
        while self.state != target_state:
            if timeout is not None:
                if (rospy.Time.now() - start_time) > timeout_duration:
                    rospy.logwarn(f"Timeout waiting for state {target_state}")
                    return False
            rospy.sleep(0.1)
        return True

    def virtual_button_tap(self, tap_num):
        self.virtual_button_pub.publish(Int32(data=tap_num))

    def load_play_list(self):
        self.play_list = SelectList()
        self.play_list.set_extract_pattern(r"teaching_(.*?)\.json")
        self.json_dir = get_cache_dir()
        for file in get_teaching_files(self.json_dir):
            self.play_list.add_option(file)

    def record_srv(self, req):
        if self.mode != "TeachingMode":
            return
        if req.data is True:
            # Start recording
            if self.wait_for_state(State.WAIT, 1) is False:
                return SetBoolResponse(success=False)
            self.virtual_button_tap(1)
            if self.wait_for_state(State.SPECIAL_ACTION_SELECT, 0.5) is True:
                # Select the first special action and start recording
                self.virtual_button_tap(2)
                if self.wait_for_state(State.RECORD, 1) is True:
                    return SetBoolResponse(success=True)
            elif self.wait_for_state(State.RECORD, 1) is True:
                # Start recording
                return SetBoolResponse(success=True)
            else:
                return SetBoolResponse(success=False)
        else:
            # Stop recording
            if self.wait_for_state(State.RECORD, 1) is False:
                return SetBoolResponse(success=False)
            self.virtual_button_tap(2)
            if self.wait_for_state(State.WAIT, 1) is False:
                return SetBoolResponse(success=False)
            else:
                return SetBoolResponse(success=True)

    def play_srv(self, req):
        """
        Execute start playing or stop playing from rosservice call.
        """
        if self.mode != "TeachingMode":
            return
        if req.data is True:
            # Start playing
            if self.wait_for_state(State.WAIT, 1) is False:
                return SelectMotionResponse(success=False)
            self.virtual_button_tap(2)
            if self.wait_for_state(State.PLAY_LIST_SELECT, 1) is False:
                return SelectMotionResponse(success=False)
            if req.name:
                ret = self.play_list.set_index_by_keyword(req.name)
                if ret is False:
                    return SelectMotionResponse(success=False)
            self.virtual_button_tap(2)  # Play the latest motion
            if self.wait_for_state(State.PLAY, 1) is False:
                return SelectMotionResponse(success=False)
        else:
            # Stop playing
            if self.wait_for_state(State.PLAY, 1) is False:
                return SelectMotionResponse(success=False)
            self.virtual_button_tap(2)
            if self.wait_for_state(State.WAIT, 3) is False:
                return SelectMotionResponse(success=False)
        return SelectMotionResponse(success=True)

    def special_action_srv(self, req):
        """
        Execute special action during record from rosservice call.
        """
        if self.mode != "TeachingMode":
            return
        if req.data is True:
            action_state = "start"
        else:
            action_state = "stop"
        if self.wait_for_state(State.RECORD, 1) is False:
            return SetBoolResponse(success=False)
        # self.virtual_button_tap(1) can only toggle.
        # self.record_and_execute_special_action can do both start and stop
        self.record_and_execute_special_action(action_state)
        if self.wait_for_state(State.RECORD, 3) is False:
            return SetBoolResponse(success=False)
        return SetBoolResponse(success=True)

    def change_name_srv(self, req):
        if self.mode != "TeachingMode":
            return
        if req.data is True:
            if self.wait_for_state(State.WAIT, 1) is False:
                return SetBoolResponse(success=False)
            self.virtual_button_tap(4)
        if self.wait_for_state(State.MOTION_LIST_SELECT, 1) is False:
            return SetBoolResponse(success=False)
        return SetBoolResponse(success=True)

    def handle_wait_state(self, msg: Int32) -> State:
        if msg.data == 1:
            # Select special action if ~special_actions param is defined
            if len(self.special_actions) > 0 and\
               self.special_action_selected is None:
                return State.SPECIAL_ACTION_SELECT
            else:
                self.start_recording()
                return State.RECORD
        elif msg.data == 2:
            return State.PLAY_LIST_SELECT
        elif msg.data == 3:
            self.teaching_manager.servo_off()
            return State.WAIT
        elif msg.data == 4:
            return State.MOTION_LIST_SELECT
        else:  # If no valid button press, do not change state
            return State.WAIT

    def handle_special_action_select_state(self, msg: Int32) -> State:
        if msg.data == 1:
            self.special_action_list.increment_index()
            return State.SPECIAL_ACTION_SELECT
        elif msg.data == 2:
            self.special_action_selected = self.special_actions[
                self.special_action_list.get_index()]
            self.start_recording()
            return State.RECORD
        else:  # If no valid button press, do not change state
            return State.SPECIAL_ACTION_SELECT

    def handle_record_state(self, msg: Int32) -> State:
        # Record and execute special action
        if msg.data == 1:
            if len(self.special_actions) == 0:
                return State.RECORD
            if self.special_action_executed is True:
                self.record_and_execute_special_action("stop")
            else:
                self.record_and_execute_special_action("start")
            return State.RECORD
        # finish recording
        elif msg.data == 2:
            self.stop_recording()
            return State.WAIT
        elif msg.data == 3:
            self.teaching_manager.servo_off()
            return State.RECORD
        else:  # If no valid button press, do not change state
            return State.RECORD

    def handle_play_list_select_state(self, msg: Int32) -> State:
        if msg.data != 0 and len(self.play_list.options) <= 0:
            return State.WAIT
        # select play file
        elif msg.data == 1:
            self.play_list.increment_index()
            return State.PLAY_LIST_SELECT
        # confirm play file
        elif msg.data == 2:
            self.start_playing(self.play_list.selected_option())
            return State.PLAY
        # delete play file
        elif msg.data == 3:
            delete_file = self.play_list.selected_option()
            if os.path.exists(delete_file):
                os.remove(delete_file)
            self.play_list.remove_option(delete_file)
            return State.WAIT
        else:  # If no valid button press, do not change state
            return State.PLAY_LIST_SELECT

    def handle_play_state(self, msg: Int32) -> State:
        # stop playing
        if msg.data == 2:
            self.stop_playing()
            return State.WAIT
        if self.playing is True:
            return State.PLAY
        else:  # If no valid button press, do not change state
            return State.WAIT

    def handle_motion_list_select_state(self, msg: Int32) -> State:
        if msg.data != 0 and len(self.play_list.options) <= 0:
            return State.WAIT
        # select motion file
        elif msg.data == 1:
            self.play_list.increment_index()
            return State.MOTION_LIST_SELECT
        # confirm motion file
        elif msg.data == 2:
            return State.CHANGE_MOTION_NAME
        else:  # If no valid button press, do not change state
            return State.MOTION_LIST_SELECT

    def handle_change_motion_name_state(self, msg: Int32) -> State:
        if self.new_motion_name is None:
            input_string = rospy.wait_for_message('~motion_name', String, timeout=None)
            self.new_motion_name = input_string.data
        if msg.data == 1:
            # retry input
            self.new_motion_name = None
            return State.CHANGE_MOTION_NAME
        if msg.data == 2:
            # confirm renaming
            self.rename_motion(self.play_list.selected_option(), self.new_motion_name)
            self.load_play_list()
            self.new_motion_name = None
            return State.WAIT
        else:  # If no valid button press, do not change state
            return State.CHANGE_MOTION_NAME

    def record_and_execute_special_action(self, action_state):
        command = self.special_action_selected[f"{action_state}_command"]
        if action_state == "stop":
            self.special_action_executed = False
        elif action_state == "start":
            self.special_action_executed = True
        else:
            rospy.logerr(f"Invalid action_state: {action_state}. Use 'stop' or 'start'")
            return
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
            if self.init_finished is True:
                # When mode is changed,
                # state automatically returns to WAIT
                self.state = State.WAIT
                self.play_list.reset_index()
            return
        delimiter = '|'
        if self.init_finished is False:
            sent_str = chr(PacketType.TEACHING_MODE)
            sent_str += delimiter
            sent_str += "Teaching Mode\n\n"
            sent_str += "Wait for servo response"
            self.write(sent_str)
            return

        sent_str = chr(PacketType.TEACHING_MODE)
        if self.teaching_manager.marker_manager.is_marker_recognized():
            marker_ids = self.teaching_manager.marker_manager.current_marker_ids()
            sent_str += str(marker_ids[0])
        sent_str += delimiter
        if self.state != State.WAIT:
            self.additional_str = ""
        if self.state == State.WAIT:
            sent_str += 'Teaching mode\n\n'\
                + '1tap: record\n'\
                + '2tap: play\n'\
                + '3tap: free'
            if self.additional_str != "":
                sent_str += "\n\n" + self.additional_str
        elif self.state == State.SPECIAL_ACTION_SELECT:
            sent_str += 'Special action\n\n'
            sent_str += ' 1tap: next\n' + ' 2tap: select\n\n'
            sent_str += self.special_action_list.string_options(5)
        elif self.state == State.RECORD:
            sent_str += 'Recording\n\n'
            # Start recording
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
        elif self.state == State.PLAY_LIST_SELECT:
            sent_str += "Play file\n"
            if len(self.play_list.options) <= 0:
                sent_str += '\nNo motion\n'\
                    + ' 1 tap: return'
            else:
                sent_str += ' 1tap: next\n'\
                    + ' 2tap: select\n'\
                    + ' 3tap: delete\n\n'
                sent_str += self.play_list.string_options(5)
        elif self.state == State.PLAY:
            sent_str += 'Playing\n\n'\
                + f'{self.play_list.selected_option(True)}\n\n'\
                + '2tap:\n stop playing'
            sent_str += f'\n\nSpeed x{self.speed}'
        elif self.state == State.MOTION_LIST_SELECT:
            sent_str += "Motion file\n"
            if len(self.play_list.options) <= 0:
                sent_str += '\nNo motion\n'\
                    + ' 1 tap: return'
            else:
                sent_str += ' 1tap: next\n'\
                    + ' 2tap: select\n\n'
                sent_str += self.play_list.string_options(5)
        elif self.state == State.CHANGE_MOTION_NAME:
            sent_str += 'Old name\n'\
                + f' {self.play_list.selected_option(True)}\n\n'\
                + 'New name\n'\
                + f' {self.new_motion_name}\n\n'\
                + '1tap:\n retry input\n'\
                + '2tap:\n confirm'
        delimiter_num = 1
        if len([char for char in sent_str if char == delimiter]) != delimiter_num:
            rospy.logerr(f"sent string: {sent_str}")
            rospy.logerr(f"The number of delimiter '{delimiter}' must be {delimiter_num}")
        self.write(sent_str)

    def additional_info_cb(self, msg):
        self.additional_str = msg.data

    def start_recording(self):
        """
        Initiates recording in a separate thread.
        """
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
        self.special_action_selected = None

    def start_playing(self, play_file):
        """
        Initiates playback in a separate thread.
        """
        def play_task():
            self.playing = True
            result_message = self.teaching_manager.play(
                play_file, self.speed)
            self.additional_str = result_message
            # Automatically stop after play
            self.stop_playing()

        self.play_thread = threading.Thread(target=play_task, daemon=True)
        self.play_thread.start()

    def stop_playing(self):
        self.teaching_manager.stop()
        self.playing = False

    def rename_motion(self, filepath, new_name):
        dir_path = os.path.dirname(filepath)
        new_filename = f"teaching_{new_name}.json"
        new_filepath = os.path.join(dir_path, new_filename)
        os.rename(filepath, new_filepath)


if __name__ == '__main__':
    # Main
    rospy.init_node('teaching_mode', anonymous=True)
    TeachingMode()
    rospy.spin()
