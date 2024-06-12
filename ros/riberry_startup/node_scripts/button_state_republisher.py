#!/usr/bin/env python3

import rospy

from std_msgs.msg import Int32, Empty
from kxr_controller.kxr_interface import KXRROSRobotInterface
from skrobot.model import RobotModel

pub_toggle = None
robot_model = None
ri = None


def callback(msg):
    global pub_toggle
    global ri
    global robot_model
    if msg.data == 11:
        pub_toggle.publish(Empty())
    elif msg.data == 5:
        ri.servo_on()
        ri.angle_vector(robot_model.init_pose(), 5)
    elif msg.data == 4:
        ri.servo_off()


if __name__ == '__main__':
    rospy.init_node('button_state_publisher')
    pub_toggle = rospy.Publisher("/robot_a/vacuum_toggle", Empty, queue_size=1)
    robot_model = RobotModel()
    namespace = ""
    robot_model.load_urdf_from_robot_description(
        namespace + '/robot_description_viz')
    ri = KXRROSRobotInterface(  # NOQA
        robot_model, namespace=namespace, controller_timeout=60.0)
    sub = rospy.Subscriber("/atom_s3_button_state", Int32, callback=callback, queue_size=1)
    rospy.spin()
