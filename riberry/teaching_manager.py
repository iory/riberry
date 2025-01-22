import json
import os

import rospy

from riberry.marker_manager import MarkerManager
from riberry.motion_manager import MotionManager


class TeachingManager:
    """A class to manage robot teaching and playback operations

    Controls the recording and playback of motion trajectories and marker information. Uses subordinate
    classes MotionManager and MarkerManager for specific motion control and marker processing.
    Recorded data is saved in JSON format and can be played back later.

    Attributes:
        motion_manager (MotionManager): Instance managing motion trajectory recording and playback
        marker_manager (MarkerManager): Instance managing marker information processing
        start_time (rospy.rostime.Time): Start time of motion
    """

    def __init__(self):
        self.motion_manager = MotionManager()
        self.marker_manager = MarkerManager()
        self.start_time = None

    def load_json(self, play_filepath):
        """Loads motion and marker data from a JSON file

        Args:
            play_filepath (str): Path to the JSON file to load

        Note:
            Loaded data is stored in motion_manager.motion and marker_manager.markers
        """

        if os.path.exists(play_filepath):
            rospy.loginfo(f'Load motion data file {play_filepath}.')
            with open(play_filepath) as f:
                json_data = json.load(f)
                self.motion_manager.set_motion([j for j in json_data if 'joint_states' in j])
                self.motion_manager.set_actions([j for j in json_data if 'special_action' in j])
                self.marker_manager.set_markers([j for j in json_data if 'marker_id' in j])
            rospy.loginfo(f'Loaded motion data: {self.motion_manager.get_motion()}')
            rospy.loginfo(f'Loaded marker data {self.marker_manager.get_markers()}')

    def stop(self):
        self.motion_manager.stop()

    def start(self):
        self.motion_manager.start()

    def servo_off(self):
        self.motion_manager.ri.servo_off()

    def servo_on(self):
        self.motion_manager.ri.servo_on()

    def record(self, record_filepath):
        """Records motion and marker information and saves it to a JSON file.

        Args:
            record_filepath (str): Path to the JSON file where data will be saved.

        Returns:
            str: Message to display on AtomS3 LCD, including recorded motion duration,
                number of motion points, and number of markers.

        Note:
            - Recording continues until `motion_manager` is stopped.
            - Data is sampled at 0.1-second intervals.
        """

        self.motion_manager.start()
        self.motion_manager.set_motion([])
        self.motion_manager.set_actions([])
        self.marker_manager.set_markers([])
        with open(record_filepath, mode='w') as f:
            rospy.loginfo(f'Start saving motion to {record_filepath}')
            while not rospy.is_shutdown():
                # Finish recording
                if self.motion_manager.is_stopped():
                    break
                if len(self.motion_manager.get_motion()) == 0:
                    self.start_time = rospy.Time.now()
                self.motion_manager.add_motion(self.start_time)
                # If self.add_motion() is already called
                if self.start_time is not None:
                    self.marker_manager.add_marker(self.start_time)
                rospy.sleep(0.1)
            f.write(
                json.dumps(
                    self.motion_manager.get_motion()+\
                    self.motion_manager.get_actions()+\
                    self.marker_manager.get_markers(),
                    indent=4, separators=(",", ": ")))
            rospy.loginfo(f'Finish saving motion to {record_filepath}')
        motion_duration = self.motion_manager.get_motion()[-1]["time"]
        message = f"Record {motion_duration:.1f} [s] motion:\n" +\
            f"{len(self.motion_manager.get_motion())} motions\n" +\
            f"{len(self.marker_manager.get_markers())} markers"
        return message

    def play(self, play_filepath, speed=1.0):
        """Plays back recorded motion

        Args:
            play_filepath (str): Path to the JSON file containing motion data to play

        Returns:
            str: Message to display on AtomS3 LCD. Returns error message if an error occurs

        Note:
            - If markers are present in the recording, the playback motion is adjusted by
            comparing current marker positions with recorded marker positions.
            - An error occurs if marker IDs don't match.
        """

        self.motion_manager.start()
        # Load all data saved in json
        self.load_json(play_filepath)
        # Play motion
        rospy.loginfo('Play motion')
        recorded_motion = self.motion_manager.get_motion()
        special_actions = self.motion_manager.get_actions()
        if len(self.marker_manager.get_markers()) == 0:
            return self.motion_manager.play_motion(
                recorded_motion, special_actions, speed)
        else:
            # The entire movement is performed again after the initial posture
            # to compensate for deflection of the arm due to gravity
            # with visual feedback.
            # self.play_motion_with_marker([self.motion_manager.get_motion()[0]])
            # # Wait for new marker topic to come after previous motion stopped
            # rospy.sleep(0.5)

            # ID check
            if not self.marker_manager.is_marker_recognized():
                error_message = "Marker must be visible at the beginning of the motion"
                rospy.logerr(error_message)
                return error_message
            first_marker = self.marker_manager.get_markers()[0]
            if first_marker["marker_id"] != self.marker_manager.current_marker_ids()[0]:
                error_message = "Current marker ID != recorded marker ID."
                rospy.logerr(error_message)
                return error_message
            # Marker coords calculation
            # TODO: Use marker at anytime
            first_marker_coords = self.marker_manager.first_marker_coords()
            current_marker_average_coords = self.marker_manager.current_marker_coords(average_num=5)
            rospy.loginfo(f"first_marker_coords: {first_marker_coords}")
            rospy.loginfo(
                f"current_marker_average_coords: {current_marker_average_coords}")
            # After recognizing marker, servo on
            self.servo_on()
            # Move motion trajectory
            moved_motion, message = self.motion_manager.move_motion(
                recorded_motion, current_marker_average_coords, first_marker_coords)
            rospy.loginfo("Note that if you record marker with servo_off" +\
                          " you should start playing with servo_off")
            if moved_motion is False:
                return message
            else:
                return self.motion_manager.play_motion(
                    moved_motion, special_actions, speed)
