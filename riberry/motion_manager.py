import copy

import numpy as np
import rospy
from skrobot.model import RobotModel
from skrobot.utils.urdf import no_mesh_load_mode

from riberry.teaching_interface import TeachingRobotInterface


class MotionManager:
    """
    This class provides functionality to record, manipulate, and execute motions for a robot
    using the `skrobot` library and ROS. It manages the robot interface, checks for compatibility
    with specific `skrobot` versions, and performs motion-related tasks, such as adding motion states,
    playing motions, and adjusting motions using inverse kinematics (IK).

    Attributes:
        ri (KXRROSRobotInterface): Interface for controlling the robot.
        joint_names (list): List of joint names for the robot.
        end_coords_name (str): Name of the end-effector coordinates link.
        motion (list): Recorded motion sequences. Each entry is a dictionary with:
            - 'time' (float): Time in seconds.
            - 'joint_states' (dict): Joint states mapping joint names to their angles (float).
        special_actions (list): Special commands and actions linked to the motion. Each entry is a dictionary with:
            - 'time' (float): Time in seconds.
            - 'special_action' (dict): Action details containing:
                - 'name' (str): Action name.
                - 'command' (str): Command string to execute.
    """

    def __init__(self):
        # skrobot version check
        from packaging import version
        import skrobot
        required_version = "0.0.45"
        current_version = skrobot.__version__
        if version.parse(current_version) < version.parse(required_version):
            raise Exception(f"skrobot version is not greater than {required_version}. (current version: {current_version})\npip install scikit-robot -U")
        # Create robot model to control pressure
        robot_model = RobotModel()
        namespace = ""
        with no_mesh_load_mode():
            robot_model.load_urdf_from_robot_description(
                namespace + "/robot_description_viz")
        self.ri = TeachingRobotInterface(
            robot_model, namespace=namespace, controller_timeout=60.0
        )
        self.joint_names = self.ri.robot.joint_names
        self.end_coords_name = rospy.get_param("~end_coords_name", None)
        link_names = [x.name for x in robot_model.link_list]
        if self.end_coords_name is not None\
           and self.end_coords_name not in link_names:
            rospy.logerr('end_coords name does not match link name.')
        self.motion = []
        self.special_actions = []
        self.start()

    def exec_with_error_handling(self, command):
        """Executes a command string with error handling.

        Args:
            command (str): Python command to execute.
        """
        try:
            exec(command)
        except Exception as e:
            rospy.logerr(f"[Special action] {e}")

    def stop(self):
        """Stops the motion execution."""
        self._stop = True

    def start(self):
        """Resumes the motion execution."""
        self._stop = False

    def is_stopped(self):
        """Checks if the motion execution is stopped.

        Returns:
            bool: True if motion execution is stopped, False otherwise.
        """
        return self._stop is True

    def set_motion(self, motion):
        """Sets the motion sequence.

        Args:
            motion (list): List of motion states.
        """
        self.motion = motion

    def get_motion(self):
        """Gets the current motion sequence.

        Returns:
            list: The current motion sequence.
        """
        return self.motion

    def add_motion(self, start_time):
        """Records the robot's current pose and adds it to the motion sequence.

        Args:
            start_time (rospy.Time): The starting time of the motion recording.
        """
        joint_states = {}
        # Average multiple angle vectors to reduce the noise
        # of the servo motor's potentiometer
        # This may cause the recorded motion to become temporally sparse.
        avs = []
        for _ in range(5):
            avs.append(self.ri.angle_vector())
        av_average = np.mean(avs, axis=0)
        for j, a in zip(self.joint_names, av_average):
            joint_states[str(j)] = float(a)
        now = rospy.Time.now()
        elapsed_time = (now - start_time).to_sec()
        self.motion.append({
            'time': elapsed_time,
            'joint_states': joint_states,
        })
        rospy.loginfo('Add new joint states')
        rospy.loginfo(f'Time: {elapsed_time}, joint_states: {joint_states}')

    def set_actions(self, actions):
        """Sets the list of special actions.

        Args:
            actions (list): List of special actions.
        """
        self.special_actions = actions

    def get_actions(self):
        """Gets the current list of special actions.

        Returns:
            list: The current special actions.
        """
        return self.special_actions

    def add_action(self, start_time, name, command):
        """Records a special action with its execution command.

        Args:
            start_time (rospy.Time): The starting time of the action recording.
            name (str): Name of the action (e.g., 'grasp').
            command (str): Python command to execute the action.
        """
        now = rospy.Time.now()
        elapsed_time = (now - start_time).to_sec()
        self.special_actions.append({
            'time': elapsed_time,
            'special_action':
            {
                'name': name,
                'command': command,
            }
        })
        rospy.loginfo('Add special action')
        rospy.loginfo(
            f'Time: {elapsed_time}, name: {name}, command: {command}')

    def parse_motion(self, motion, actions):
        """
        ロボットの動作データを処理し、joint_statesとspecial_actionで分類する関数

        Args:
            data: JSON形式のロボット動作データ

        Returns:
            処理済みのデータを含む辞書
        """
        sorted_motion = sorted(motion, key=lambda x: x['time'])
        sorted_actions = sorted(actions, key=lambda x: x['time'])
        special_actions = []
        joint_states = []
        for item in sorted_motion:
            if 'joint_states' not in item:
                rospy.logerr("Cannot parse data. Invalid motion is given.")
                return
            joint_states.append(item)
        for item in sorted_actions:
            if 'special_action' not in item:
                rospy.logerr("Cannot parse data. Invalid actions are given.")
                return
            special_actions.append(item)
        # special_actionの時間でjoint_statesを分割
        motion_segments = []
        current_segment = []
        special_action_times = [action['time'] for action in special_actions]
        for state in joint_states:
            time = state['time']
            # 現在のsegmentを決定
            should_start_new_segment = False
            if current_segment:
                for special_time in special_action_times:
                    if time > special_time and current_segment[-1]['time'] < special_time:
                        should_start_new_segment = True
                        break
            if should_start_new_segment:
                motion_segments.append(current_segment)
                current_segment = []
            current_segment.append(state)
        # 最後のセグメントを追加
        if current_segment:
            motion_segments.append(current_segment)
        if len(motion_segments) != len(special_actions) + 1:
            rospy.logerr("Parse failed.")
            return
        return {
            "motion_segments": motion_segments,
            "special_actions": special_actions
        }

    def play_motion(self, motion, actions, speed=1.0):
        """Executes a sequence of motions with safety checks and interruption handling.

        Returns:
            message (str): Message shown on AtomS3 LCD
        """
        # To prevent sudden movement, take time to reach the initial motion
        if 'joint_states' not in motion[0]:
            print("First element must have 'joint_states' key.")
            return
        if speed == 0:
            print("Speed cannot be zero. Setting speed to 1.0.")
            speed = 1.0
        self.ri.servo_on()
        first_av = list(motion[0]['joint_states'].values())
        diff_av = first_av - self.ri.angle_vector()
        max_diff_angle = max(map(abs, diff_av))
        first_time = max_diff_angle / (np.pi/4)
        print(f"Move to the initial position in {first_time} seconds")
        self.ri.angle_vector(first_av, first_time)
        self.ri.wait_interpolation()
        # Play the actions from the second one onward
        prev_time = motion[0]['time']
        _ = self.parse_motion(motion[1:], actions)
        motion_segments = _["motion_segments"]
        special_actions = _["special_actions"]
        # Separate motions by special action and play them in turns
        for i, motion_segment in enumerate(motion_segments):
            avs = []
            tms = []
            filtered_motion_segment = [m for m in motion_segment
                                       if m['time'] >= prev_time]
            # Play motion segment
            for m in filtered_motion_segment:
                current_time = m['time']
                # Send angle vector
                if 'joint_states' in m:
                    av = np.array(list(m['joint_states'].values()))
                    avs.append(av)
                    tms.append((current_time - prev_time) / speed)
                prev_time = current_time
            rospy.loginfo('angle vectors')
            rospy.loginfo(f'{avs}')
            rospy.loginfo('times')
            rospy.loginfo(f'{tms}')
            self.ri.angle_vector_sequence(avs, tms)
            # Check interruption by button
            while not rospy.is_shutdown() and self.ri.is_interpolating():
                if self.is_stopped():
                    self.ri.cancel_angle_vector()
                    break
                rospy.sleep(0.5)  # Save motion every 0.5s to smooth motion play
            if self.is_stopped():
                rospy.loginfo('Play interrupted')
                break
            # Play special action
            start_time = rospy.Time.now()
            if i < len(special_actions):
                action = special_actions[i]["special_action"]
                rospy.loginfo('Run special action')
                rospy.loginfo(
                    f'name: {action["name"]}, command: {action["command"]}')
                self.exec_with_error_handling(
                    action["command"])
            elapsed_time = (rospy.Time.now() - start_time).to_sec()
            # Skip motions recorded during special action
            prev_time += elapsed_time
        message = 'Play finished'
        rospy.loginfo(message)
        return message

    def move_motion(self, motion, target_coords, local_coords):
        """Adjusts a motion sequence to align with a specified target pose using inverse kinematics (IK).

        This method modifies a given motion sequence by applying IK to align the end-effector
        with a specified target position and orientation. If IK fails consecutively beyond
        a certain limit, the process terminates and returns an error message.

        Args:
            motion (list): A list of motion states.
            target_coords (Coords): Target coordinates for the end-effector in the global frame.
            local_coords (Coords): Local offset coordinates relative to the target position.

        Returns:
            tuple: A tuple containing:
                - moved_motion (list or bool): Updated motion sequence if successful, or False if IK fails.
                - message (str): Success or error message.
        """
        if self.end_coords_name is None:
            error_message = "end_coords_name param is not set."
            rospy.logerr(error_message)
            return (False, error_message)
        robot = copy.deepcopy(self.ri.robot)
        end_coords = getattr(robot, self.end_coords_name)
        # Calculate target coords
        moved_motion = copy.deepcopy(motion)
        consecutive_false_count = 0
        false_count_limit = min(5, len(moved_motion))
        failure_indices = []
        for i, m in enumerate(moved_motion):
            joint_states = m["joint_states"]
            for joint_name in joint_states.keys():
                getattr(robot, joint_name).joint_angle(joint_states[joint_name])
            ik_coords = end_coords.copy_worldcoords()
            ik_coords.move_coords(target_coords, local_coords)
            # Calculated moved motion
            link_list = robot.link_list
            link_list_has_joint = [x for x in link_list if x.joint is not None]
            ret = robot.inverse_kinematics(
                ik_coords, link_list=link_list_has_joint, move_target=end_coords,
                # thre=[0.001 * 5], rthre=[np.deg2rad(1*5)],  # 5x times loose IK  # spellchecker:disable-line
                stop=10,  # faster IK
            )
            if isinstance(ret, np.ndarray) is False:
                rospy.logwarn('IK failed')
                failure_indices.append(i)
                consecutive_false_count += 1
                if consecutive_false_count == false_count_limit:
                    error_message = f"IK failed {false_count_limit} consecutive times."
                    rospy.logerr(error_message)
                    return (False, error_message)
            else:
                consecutive_false_count = 0
            # Overwrite moved_motion
            for joint_name in joint_states.keys():
                joint_states[joint_name] = getattr(robot, joint_name).joint_angle()
        rospy.loginfo(f"failure indices: {failure_indices}" +\
                      f" in {len(moved_motion)} trajectories")
        for j in sorted(failure_indices, reverse=True):
            del moved_motion[j]
        message = "IK success"
        rospy.loginfo(message)
        return (moved_motion, message)
