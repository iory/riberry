from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
import numpy as np
import rospy
from skrobot.coordinates.base import Coordinates
from skrobot.coordinates.base import transform_coords
from skrobot.interfaces.ros.tf_utils import geometry_pose_to_coords
from skrobot.interfaces.ros.tf_utils import tf_pose_to_coords
from skrobot.interfaces.ros.transform_listener import TransformListener


class MarkerManager:
    """
    The `MarkerManager` class manages AprilTag detection data and provides marker-related information for robot operations.

    This class uses ROS to provide the following functionalities:
    1. Subscribes to AprilTag detection results and manages the data.
    2. Retrieves IDs, positions, and orientations of detected markers.
    3. Computes the coordinates of the first detected marker or the current marker and returns them as `Coordinates` from the `skrobot` library.
    4. Offers a utility method for averaging quaternions.

    Attributes:
        tfl (TransformListener): An instance of ROS TransformListener used for retrieving TF transformations.
        markers (list): A list of previously recorded marker information.
        marker_msg (AprilTagDetectionArray): The latest marker detection data.
    """

    def __init__(self):
        # Marker
        rospy.Subscriber(
            "tag_detections", AprilTagDetectionArray,
            callback=self.apriltag_cb, queue_size=1)
        self.tfl = TransformListener(use_tf2=False) # Need 0.5[s] wait to initialize
        self.markers = []
        self.marker_msg = None

    def apriltag_cb(self, msg):
        """Callback function to handle AprilTag detection data.

"""
        self.marker_msg = msg

    def is_marker_recognized(self):
        """Returns whether any markers are currently detected.

"""
        return self.marker_msg is not None and len(self.marker_msg.detections) > 0

    def current_marker_ids(self):
        """Returns a list of IDs for the currently detected markers.

"""
        return [detection.id[0] for detection in self.marker_msg.detections]

    def set_markers(self, markers):
        self.markers = markers

    def get_markers(self):
        return self.markers

    def add_marker(self, start_time):
        """Records the currently detected marker's data.

"""
        # No marker message has come
        if not self.is_marker_recognized():
            return
        elapsed_time = self.marker_msg.header.stamp - start_time
        elapsed_time = elapsed_time.to_sec()
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

    def first_marker_coords(self, average_num=5):
        """Computes and returns the coordinates of the first recorded marker.

"""
        # Calculate first recorded marker coords from base_link
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
        return first_marker_coords

    def current_marker_coords(self, average_num=5):
        """Computes and returns the coordinates of the currently detected marker, averaged over multiple measurements.

"""
        # Calculate current marker coords from base_link
        # Average multiple angle vectors to reduce the noise
        # of the servo motor's potentiometer
        # This may cause the delay before playing motion
        positions = [] # x, y, z
        quaternions = []  # x, y, z, w
        for _ in range(average_num):
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
            rot=self.quaternion_average_slerp(quaternions)
        )
        return current_marker_average_coords

    def quaternion_average_slerp(self, quaternions):
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
