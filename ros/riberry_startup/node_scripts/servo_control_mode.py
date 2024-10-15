#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import String

from kxr_controller.kxr_interface import KXRROSRobotInterface
from kxr_controller.msg import ServoOnOff
from skrobot.model import RobotModel

from riberry.i2c_base import I2CBase


class ServoControlMode(I2CBase):
    def __init__(self, i2c_addr, lock_path='/tmp/i2c_servo_control_mode.lock'):
        super().__init__(i2c_addr)
        # Create robot model to control servo
        robot_model = RobotModel()
        namespace = ""
        robot_model.load_urdf_from_robot_description(
            namespace + '/robot_description_viz')
        self.ri = KXRROSRobotInterface(  # NOQA
            robot_model, namespace=namespace, controller_timeout=60.0)

        # Button and mode callback
        self.mode = None
        rospy.Subscriber(
            "/atom_s3_button_state", Int32,
            callback=self.button_cb, queue_size=1)
        rospy.Subscriber(
            "/atom_s3_mode", String,
            callback=self.mode_cb, queue_size=1)

        # Servo on off
        self.servo_on_states = None
        rospy.Subscriber(
            "/kxr_fullbody_controller/servo_on_off_real_interface/state",
            ServoOnOff, callback=self.servo_on_off_cb, queue_size=1)

        rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def button_cb(self, msg):
        """
        When AtomS3 is ServoControlMode and single-click pressed,
        toggle servo control.
        """
        if self.mode == 'ServoControlMode' and msg.data == 1:
            rospy.loginfo(
                'AtomS3 is ServoControlMode and single-click-pressed.'
                ' Toggle servo control.')
            self.toggle_servo_on_off()

    def mode_cb(self, msg):
        """
        Check AtomS3 mode.
        """
        self.mode = msg.data

    def servo_on_off_cb(self, msg):
        self.servo_on_states = msg

    def toggle_servo_on_off(self):
        """
        If one of the servos is on, turn the entire servo off.
        If all of the servos is off, turn the entire servo on.
        """
        if self.ri is None:
            rospy.logwarn("KXRROSRobotInterface instance is not created.")
            return
        if self.servo_on_states is None:
            return
        servo_on_states = self.servo_on_states.servo_on_states
        if any(servo_on_states) is True:
            self.ri.servo_off()
        else:
            self.ri.servo_on()

    def timer_callback(self, event):
        if self.mode != 'ServoControlMode':
            return
        # Send message on AtomS3 LCD
        sent_str = 'Servo control mode\n\nSingle Click:\nServo on off'
        self.send_string(sent_str)


if __name__ == '__main__':
    rospy.init_node('servo_control_mode')
    scm = ServoControlMode(0x42)
    rospy.spin()
