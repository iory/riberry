#!/usr/bin/env python3

import threading

import rospy
from std_msgs.msg import Int32, Empty
from kxr_controller.kxr_interface import KXRROSRobotInterface
from skrobot.model import RobotModel


class PumpOnOff(object):
    """
    Pressing and holding AtomS3 toggles the pump on and off.
    """
    def __init__(self):
        self.pub_toggle = rospy.Publisher(
            "/robot_a/vacuum_toggle", Empty, queue_size=1)
        rospy.Subscriber(
            "/atom_s3_button_state", Int32, callback=self.cb, queue_size=1)

    def cb(self, msg):
        # When AtomS3 is pressed and holded
        if msg.data == 11:
            self.pub_toggle.publish(Empty())

       
class ServoOnOff(threading.Thread):
    """
    4 clicks to turn servo off, 5 clicks to turn servo on.
    The process may stop when creating a KXRROSRobotInterface instance.
    So the process was threaded so that it would not stop the main process.
    """
    def __init__(self):
        super().__init__(daemon=True)
        self.ri = None
        rospy.Subscriber(
            "/atom_s3_button_state", Int32, callback=self.cb, queue_size=1)
        self.start()

    def cb(self, msg):
        state = msg.data
        if state == 4 or state == 5:
            if self.ri is None:
                rospy.logwarn("KXRROSRobotInterface instance is not created.")
                return
            if state == 4:
                self.ri.servo_off()
            elif state == 5:
                self.ri.servo_on()

    def run(self):
        """
        self.run() is executed when the self.start() is called.
        """
        robot_model = RobotModel()
        namespace = ""
        robot_model.load_urdf_from_robot_description(
            namespace + '/robot_description_viz')
        self.ri = KXRROSRobotInterface(  # NOQA
            robot_model, namespace=namespace, controller_timeout=60.0)


if __name__ == "__main__":
    """
    All AtomS3 clicking actions are integrated into this program
    to prevent different actions from being assigned to the same click.
    """
    rospy.init_node('button_action_manager')
    PumpOnOff()
    ServoOnOff()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Finish handle_button_press")
