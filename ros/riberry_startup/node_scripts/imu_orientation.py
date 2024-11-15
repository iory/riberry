#!/usr/bin/env python3
import numpy as np
from riberry_startup.msg import ImuFace
import rospy
from sensor_msgs.msg import Imu


class IMUOrientation:
    def __init__(self):
        self.imu_sub = rospy.Subscriber("~imu", Imu, self.imu_callback)
        self.imu_orientation_pub = rospy.Publisher(
            "~imu_face", ImuFace, queue_size=1)
        self.threshold = rospy.get_param("~threshold", 0.8)

    def imu_callback(self, msg):
        acc_x = msg.linear_acceleration.x
        acc_y = msg.linear_acceleration.y
        acc_z = msg.linear_acceleration.z

        # Normalize acceleration vector
        acc = np.array([acc_x, acc_y, acc_z])
        norm = np.linalg.norm(acc)
        if norm == 0:
            rospy.logwarn("Zero acceleration vector received.")
            msg = ImuFace(face=ImuFace.NONE)
            self.imu_orientation_pub.publish(msg)
            return
        acc_normalized = acc / norm

        # Select a closest face by calculating inner product
        faces = {
            ImuFace.TOP: np.array([0, 0, 1]),
            ImuFace.BOTTOM: np.array([0, 0, -1]),
            ImuFace.FRONT: np.array([0, 1, 0]),
            ImuFace.BACK: np.array([0, -1, 0]),
            ImuFace.LEFT: np.array([1, 0, 0]),
            ImuFace.RIGHT: np.array([-1, 0, 0]),
        }

        best_match = None
        best_similarity = -1
        for face, direction in faces.items():
            similarity = np.dot(acc_normalized, direction)
            if similarity > best_similarity and similarity > self.threshold:
                best_match = face
                best_similarity = similarity
        if best_match is None:
            best_match = ImuFace.NONE

        msg = ImuFace(face=best_match)
        self.imu_orientation_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("imu_orientation")
    imu_orientation = IMUOrientation()
    rospy.spin()
