#!/usr/bin/env python3

from apriltag_ros.msg import AprilTagCornerDetectionArray
from apriltag_ros.msg import AprilTagDetection
from apriltag_ros.msg import AprilTagDetectionArray
from cameramodels import PinholeCameraModel
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
import message_filters
import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from skrobot.coordinates.base import Coordinates
from skrobot.coordinates.math import rotation_matrix_from_axis
from skrobot.interfaces.ros.tf_utils import coords_to_geometry_pose


class MarkerDepthPoseEstimator:

    def __init__(self):
        super().__init__()

        self.bridge = CvBridge()
        self.camera_info_msg = None
        self.camera_model = None

        self.pub = rospy.Publisher(
            "~output", AprilTagDetectionArray, queue_size=1)
        self.pub_debug = rospy.Publisher(
            "~debug", PoseStamped, queue_size=1)

        self.subscribe()

    def subscribe(self):
        queue_size = rospy.get_param('~queue_size', 10)
        self.standalone_tags_info = rospy.get_param('~standalone_tags')
        sub_corner_detections = message_filters.Subscriber(
            '~input/tag_corner_detections',
            AprilTagCornerDetectionArray, queue_size=1)
        sub_depth = message_filters.Subscriber(
            '~input/depth',
            Image, queue_size=1, buff_size=2**24)
        self.subs = [sub_corner_detections, sub_depth]

        self.camera_types = ["color", "depth"]
        self.camera_info_msg = {}
        self.camera_model = {}
        self.sub_info = {}
        for camera_type in self.camera_types:
            self.camera_info_msg[camera_type] = None
            self.camera_model[camera_type] = None
            self.sub_info[camera_type] = rospy.Subscriber(
                '~input/' + camera_type + '/info',
                CameraInfo, self._cb_cam_info, camera_type)

        if rospy.get_param('~approximate_sync', True):
            slop = rospy.get_param('~slop', 0.1)
            sync = message_filters.ApproximateTimeSynchronizer(
                fs=self.subs, queue_size=queue_size, slop=slop)
        else:
            sync = message_filters.TimeSynchronizer(
                fs=self.subs, queue_size=queue_size)
        sync.registerCallback(self._cb_with_depth)

    def unsubscribe(self):
        self.sub.unregister()

    def _cb_cam_info(self, msg, camera_type):
        self.camera_info_msg[camera_type] = msg
        self.camera_model[camera_type] = PinholeCameraModel.from_camera_info(
            self.camera_info_msg[camera_type])
        self.sub_info[camera_type].unregister()
        self.sub_info[camera_type] = None
        rospy.loginfo("Received camera info")

    def _cb_with_depth(self, corner_detections_msg, depth_img_msg):
        for camera_type in self.camera_types:
            if self.camera_model[camera_type] is None:
                rospy.logwarn(f'waiting {camera_type} camera info message.')
                return
        bridge = self.bridge
        supported_encodings = {'16UC1', '32FC1'}
        if depth_img_msg.encoding not in supported_encodings:
            rospy.logwarn(
                f'Unsupported depth image encoding: {depth_img_msg.encoding}')

        depth = bridge.imgmsg_to_cv2(depth_img_msg)
        if depth_img_msg.encoding == '16UC1':
            depth = depth / 1000.0  # convert metric: mm -> m

        # Publish marker estimated with depth image
        header = corner_detections_msg.header
        pub_msg = AprilTagDetectionArray(header=header)
        AprilTagCornerDetectionArray(header=header)
        for detection in corner_detections_msg.detections:
            corners = np.array(detection.corners, dtype=np.int32).reshape(-1, 2)
            corners_in_depth_img = self.batch_transform_pixel(
                self.camera_model["color"],
                self.camera_model["depth"],
                corners)
            corners_in_depth_img = np.round(corners_in_depth_img).astype("int")
            depth_values = depth[tuple(corners_in_depth_img[:, [1, 0]].T)]
            # Corner points calculated in depth image
            corners_3d = self.camera_model["depth"].batch_project_pixel_to_3d_ray(
                corners_in_depth_img,
                depth_values
            )

            # Check depth estimation accuracy by comparing
            # marker_size_in_depth_img and actual_marker_size
            actual_marker_size = None
            for marker_info in self.standalone_tags_info:
                if marker_info["id"] == detection.id[0]:
                    actual_marker_size = marker_info["size"]
            if actual_marker_size is None:
                rospy.logerr("tag data is not defined")
                return
            marker_size_in_depth_img = np.linalg.norm(
                np.roll(corners_3d, -1, axis=0) - corners_3d, axis=1, ord=2)
            tolerance_percent = 0.05
            marker_size_error = self.marker_size_error(
                marker_size_in_depth_img,
                actual_marker_size
            )
            if marker_size_error > tolerance_percent:
                rospy.logerr(f"Marker({detection.id[0]}) depth estimation fail. " +\
                             "Marker size error " +\
                             f"is {marker_size_error*100:.1f}% " +\
                             f"(> {tolerance_percent*100:.1f}%)"
                )
                # For debug
                # rospy.logerr("Corner points calculated in depth image:\n"+\
                #              f"{corners_3d}")
                # rospy.logerr(
                #     f"Corner to Corner distances: {marker_size_in_depth_img}")
                return
            else:
                rospy.loginfo(f"Marker({detection.id[0]}) depth estimation succeed. " +\
                              "Marker size error " +\
                              f"is {marker_size_error*100:.1f}% " +\
                              f"(< {tolerance_percent*100:.1f}%)"
                )

            # Publish marker info with good accuracy
            detection_with_depth = AprilTagDetection(
                id = detection.id,
                size = [actual_marker_size])
            marker_pose_with_depth = self.corners_to_pose(corners_3d)
            self.pub_debug.publish(
                PoseStamped(
                    header=header,
                    pose=marker_pose_with_depth))
            detection_with_depth.pose.pose.pose = marker_pose_with_depth
            pub_msg.detections.append(detection_with_depth)
        self.pub.publish(pub_msg)

    def batch_transform_pixel(self, camera1, camera2, pixels1):
        """
        Batch transform pixel coordinates from camera1 to camera2.

        Parameters:
        - camera1: PinholeCameraModel instance for the first camera.
        - camera2: PinholeCameraModel instance for the second camera.
        - pixels1: numpy.ndarray of shape (N, 2), where N is the number of pixels.
                   Each row represents a pixel coordinate (x1, y1) in camera1.

        Returns:
        - pixels2: numpy.ndarray of shape (N, 2), representing the corresponding
                   pixel coordinates (x2, y2) in camera2.
        """
        # Step 1: Project the batch of pixels from camera1 to 3D rays
        rays = camera1.batch_project_pixel_to_3d_ray(pixels1)

        # Step 2: Assume all points lie on the image plane (z=1)
        points_3d = rays * np.array([1, 1, 1])

        # Step 3: Project the 3D points onto camera2's image plane
        pixels2 = camera2.batch_project3d_to_pixel(points_3d)

        return pixels2

    def marker_size_error(self, arr, marker_size):
        """
        Check if any array element is outside marker_size ± tolerance range

        Args:
            arr: numpy array of values to check. Unit is [m].
            marker_size: Actual marker size. Unit is [m].

        Returns:
            float: Percentage of error
        """
        lower_error = 1.0 - np.min(arr) / float(marker_size)
        upper_error = np.max(arr) / float(marker_size) - 1.0
        return max(lower_error, upper_error)

    def corners_to_pose(self, corners_3d):
        """
        Returns
        -------
        pose : geometry_msgs.msg.Pose
"""
        x_axis1 = corners_3d[1] - corners_3d[2]
        x_axis2 = corners_3d[0] - corners_3d[3]
        x_axis = (x_axis1 + x_axis2) / 2.0
        y_axis1 = corners_3d[1] - corners_3d[0]
        y_axis2 = corners_3d[2] - corners_3d[3]
        y_axis = (y_axis1 + y_axis2) / 2.0
        m = rotation_matrix_from_axis(x_axis, y_axis)
        c = Coordinates(pos = np.mean(corners_3d, axis=0),
                        rot = m)
        return coords_to_geometry_pose(c)

if __name__ == '__main__':
    rospy.init_node('marker_depth_pose_estimator')
    node = MarkerDepthPoseEstimator()
    rospy.spin()
