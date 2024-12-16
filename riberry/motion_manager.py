import copy
import json
import os

from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from kxr_controller.kxr_interface import KXRROSRobotInterface
import numpy as np
import rospy
from skrobot.coordinates.base import Coordinates
from skrobot.coordinates.base import transform_coords
from skrobot.interfaces.ros.tf_utils import geometry_pose_to_coords
from skrobot.interfaces.ros.tf_utils import tf_pose_to_coords
from skrobot.interfaces.ros.transform_listener import TransformListener
from skrobot.model import RobotModel
from skrobot.utils.urdf import no_mesh_load_mode


def quaternion_average_slerp(quaternions):
    """
    球面線形補間Slerpを使用したクォータニオンの平均計算

    Parameters:
    quaternions : numpy.ndarray
        平均を計算するクォータニオンの配列 (N x 4)
        w, x, y, z の順

    Returns:
    numpy.ndarray
        平均クォータニオン (w, x, y, z)
    """
    # 最初のクォータニオンを基準として使用
    reference = quaternions[0]
    # すべてのクォータニオンを同じ半球に射影
    sign = np.sign(np.dot(quaternions, reference))
    quaternions *= sign[:, np.newaxis]
    # 重み付け平均。同じweightを想定
    mean_quat = np.mean(quaternions, axis=0)
    # 平均クォータニオンを正規化
    return mean_quat / np.linalg.norm(mean_quat)


class MotionManager:
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
        self.start_time = None
        self.motion = []
        self.tfl = TransformListener(use_tf2=False) # Need 0.5[s] wait to initialize
        # TODO: default value
        self.end_coords_name = rospy.get_param("~end_coords_name", None)
        link_names = [x.name for x in robot_model.link_list]
        if self.end_coords_name is not None\
           and self.end_coords_name not in link_names:
            rospy.logerr('end_coords name does not match link name.')
        # Marker
        rospy.Subscriber(
            "tag_detections", AprilTagDetectionArray,
            callback=self.marker_cb, queue_size=1)
        self.markers = []
        self.marker_msg = None
        self._stop = False

    def add_motion(self):
        """
        Add current pose to self.motion
"""
        if len(self.motion) == 0:
            self.start_time = rospy.Time.now()
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
        elapsed_time = (now - self.start_time).to_sec()
        self.motion.append({
            'time': elapsed_time,
            'joint_states': joint_states,
        })
        rospy.loginfo('Add new joint states')
        rospy.loginfo(f'Time: {elapsed_time}, joint_states: {joint_states}')

    def marker_cb(self, msg):
        self.marker_msg = msg

    def add_marker(self):
        # self.add_motion() is not called yet
        if self.start_time is None:
            return
        # No marker message has come
        if self.marker_msg is None or len(self.marker_msg.detections) == 0:
            return
        now = rospy.Time.now()
        elapsed_time = (now - self.start_time).to_sec()
        marker = self.marker_msg.detections[0]
        marker_position = marker.pose.pose.pose.position
        marker_orientation = marker.pose.pose.pose.orientation
        self.markers.append({
            'time': elapsed_time,
            'frame_id': marker.pose.header.frame_id,
            'marker_id': marker.id[0],
            'marker_size': marker.size[0],
            'position': [marker_position.x, marker_position.y, marker_position.z],
            'orientation': [marker_orientation.x, marker_orientation.y, marker_orientation.z, marker_orientation.w],
        })
        rospy.loginfo('Add new marker')
        rospy.loginfo(f'Time: {elapsed_time}, marker: {marker}')

    def stop(self):
        self._stop = True

    def servo_off(self):
        self.ri.servo_off()

    def record(self, record_filepath):
        """
Returns message[str] to show on AtomS3 LCD
"""
        self._stop = False
        self.motion = []
        self.markers = []
        with open(record_filepath, mode='w') as f:
            rospy.loginfo(f'Start saving motion to {record_filepath}')
            while not rospy.is_shutdown():
                # Finish recording
                if self._stop is True:
                    break
                self.add_motion()
                self.add_marker()
                rospy.sleep(0.1)
            f.write(json.dumps(self.motion+self.markers, indent=4, separators=(",", ": ")))
            rospy.loginfo(f'Finish saving motion to {record_filepath}')
        motion_duration = self.motion[-1]["time"]
        message = f"Record {motion_duration:.1f} [s] motion:\n" +\
            f"{len(self.motion)} motions\n" +\
            f"{len(self.markers)} markers"
        return message

    def load_json(self, play_filepath):
        # Load motion
        if os.path.exists(play_filepath):
            rospy.loginfo(f'Load motion data file {play_filepath}.')
            with open(play_filepath) as f:
                json_data = json.load(f)
                self.motion = [j for j in json_data if 'joint_states' in j]
                self.markers = [j for j in json_data if 'marker_id' in j]
            rospy.loginfo(f'Loaded motion data: {self.motion}')
            rospy.loginfo(f'Loaded marker data {self.markers}')

    def play_motion(self, motion):
        """
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
            if self._stop is True:
                self.ri.cancel_angle_vector()
                rospy.loginfo('Play interrupted')
                break
            rospy.sleep(0.5)  # Save motion every 0.5s to smooth motion play
        message = 'Play finished'
        rospy.loginfo(message)
        return message

    def move_motion(self, motion, target_coords, local_coords):
        """
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

    def play_motion_with_marker(self, motion):
        """
Returns message[str] to show on AtomS3 LCD
"""
        # Calculate first recorded marker coords from base_link
        # TODO: Use marker at anytime
        first_marker = self.markers[0]
        # if first_marker["time"] > 1:
        #     error_message = "Marker must be recorded at the beginning of the motion"
        #     rospy.logerr(error_message)
        #     return error_message
        camera_to_first_marker_pose = Pose(
            position=Point(x=first_marker["position"][0],
                           y=first_marker["position"][1],
                           z=first_marker["position"][2]),
            orientation=Quaternion(x=first_marker["orientation"][0],
                                   y=first_marker["orientation"][1],
                                   z=first_marker["orientation"][2],
                                   w=first_marker["orientation"][3]))
        base_to_camera_tf = self.tfl.lookup_transform(
            "base_link", first_marker["frame_id"], rospy.Time(0))
        first_marker_coords = transform_coords(
            tf_pose_to_coords(base_to_camera_tf),
            geometry_pose_to_coords(camera_to_first_marker_pose))
        # Calculate current marker coords from base_link
        # Average multiple angle vectors to reduce the noise
        # of the servo motor's potentiometer
        # This may cause the delay before playing motion
        positions = [] # x, y, z
        quaternions = []  # x, y, z, w
        for _ in range(5):
            if self.marker_msg is None or len(self.marker_msg.detections) == 0:
                error_message = "Marker must be visible at the beginning of the motion"
                rospy.logerr(error_message)
                return error_message
            if first_marker["marker_id"] != self.marker_msg.detections[0].id[0]:
                error_message = "Current marker ID != recorded marker ID."
                rospy.logerr(error_message)
                return error_message
            current_marker = self.marker_msg.detections[0]
            base_to_camera_tf = self.tfl.lookup_transform(
                "base_link", current_marker.pose.header.frame_id,
                rospy.Time(0))
            camera_to_current_marker_pose = current_marker.pose.pose.pose
            current_marker_coords = transform_coords(
                tf_pose_to_coords(base_to_camera_tf),
                geometry_pose_to_coords(camera_to_current_marker_pose))
            positions.append(current_marker_coords.worldpos())
            quaternions.append(current_marker_coords.quaternion_wxyz)
            rospy.sleep(0.1)
        current_marker_average_coords = Coordinates(
            pos=np.mean(positions, axis=0),
            rot=quaternion_average_slerp(quaternions)
            )
        # After recognizing marker, servo on
        self.ri.servo_on()
        moved_motion, message = self.move_motion(
            motion, current_marker_average_coords, first_marker_coords)
        rospy.loginfo(f"current_marker_coords: {current_marker_coords}")
        rospy.loginfo(
            f"first_marker_coords: {current_marker_average_coords}")
        if moved_motion is False:
            return message
        else:
            return self.play_motion(moved_motion)

    def play(self, play_filepath):
        self._stop = False
        # Load all data saved in json
        self.load_json(play_filepath)
        # Play motion
        rospy.loginfo('Play motion')
        if len(self.markers) == 0:
            return self.play_motion(self.motion)
        else:
            # The entire movement is performed again after the initial posture
            # to compensate for deflection of the arm due to gravity
            # with visual feedback.
            # self.play_motion_with_marker([self.motion[0]])
            # # Wait for new marker topic to come after previous motion stopped
            # rospy.sleep(0.5)

            rospy.loginfo("Note that if you record marker with servo_off" +\
                          " you should start playing with servo_off")
            return self.play_motion_with_marker(self.motion)
