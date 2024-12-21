import copy

from kxr_controller.kxr_interface import KXRROSRobotInterface
import numpy as np
import rospy
from skrobot.model import RobotModel
from skrobot.utils.urdf import no_mesh_load_mode


class MotionManager:
    """
    This class provides functionality to record, manipulate, and execute motions for a robot
    using the `skrobot` library and ROS. It manages the robot interface, checks for compatibility
    with specific `skrobot` versions, and performs motion-related tasks, such as adding motion states,
    playing motions, and adjusting motions using inverse kinematics (IK).

    Attributes:
        tfl (TransformListener): An instance of ROS TransformListener used for retrieving TF transformations.
        markers (list): A list of previously recorded marker information.
        marker_msg (AprilTagDetectionArray): The latest marker detection data.
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
        self.ri = KXRROSRobotInterface(
            robot_model, namespace=namespace, controller_timeout=60.0
        )
        self.joint_names = self.ri.robot.joint_names
        self.end_coords_name = rospy.get_param("~end_coords_name", None)
        link_names = [x.name for x in robot_model.link_list]
        if self.end_coords_name is not None\
           and self.end_coords_name not in link_names:
            rospy.logerr('end_coords name does not match link name.')
        self.motion = []
        self.start()

    def stop(self):
        """Stops the motion execution.

"""
        self._stop = True

    def start(self):
        """Resumes the motion execution.

"""
        self._stop = False

    def is_stopped(self):
        """Checks if the motion execution is stopped.

"""
        return self._stop is True

    def set_motion(self, motion):
        self.motion = motion

    def get_motion(self):
        return self.motion

    def add_motion(self, start_time):
        """Records the robot's current pose and adds it to the motion sequence.

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

    def play_motion(self, motion):
        """Executes a sequence of motions with safety checks and interruption handling.

Returns message[str] to show on AtomS3 LCD
"""
        # To prevent sudden movement, take time to reach the initial motion
        if 'joint_states' not in motion[0]:
            print("First element must have 'joint_states' key.")
            return
        self.ri.servo_on()
        first_av = list(motion[0]['joint_states'].values())
        self.ri.angle_vector(first_av, 3)
        self.ri.wait_interpolation()
        # Play the actions from the second one onward
        prev_time = motion[0]['time']
        avs = []
        tms = []
        for m in motion[1:]:
            current_time = m['time']
            # Send angle vector
            if 'joint_states' in m:
                av = np.array(list(m['joint_states'].values()))
                avs.append(av)
                tms.append(current_time - prev_time)
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
                rospy.loginfo('Play interrupted')
                break
            rospy.sleep(0.5)  # Save motion every 0.5s to smooth motion play
        message = 'Play finished'
        rospy.loginfo(message)
        return message

    def move_motion(self, motion, target_coords, local_coords):
        """Adjusts a motion sequence to align with a specified target pose using inverse kinematics (IK).
Returns (moved_motion[json] or False, message[str])
if IK succeed, moved_motion is [json].
if IK fail, moved_motion is False.
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
                # thre=[0.001 * 5], rthre=[np.deg2rad(1*5)],  # 5x times loose IK
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
