#!/usr/bin/env python3

from kxr_controller.kxr_interface import KXRROSRobotInterface
from kxr_controller.msg import ServoOnOff
import rospy
from skrobot.model import RobotModel
from skrobot.utils.urdf import no_mesh_load_mode
from std_msgs.msg import Int32

from riberry.com.base import PacketType
from riberry.mode import Mode


class ServoControlMode(Mode):
    def __init__(self):
        super().__init__()
        # Create timer callback first to send string even if self.ri cannot be generated
        self.init_finished = False
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)

        # Create robot model to control servo
        robot_model = RobotModel()
        namespace = ""
        with no_mesh_load_mode():
            robot_model.load_urdf_from_robot_description(
                namespace + "/robot_description_viz")
        self.ri = KXRROSRobotInterface(
            robot_model, namespace=namespace, controller_timeout=60.0
        )

        # Button and mode callback
        rospy.Subscriber(
            "atom_s3_button_state", Int32, callback=self.button_cb, queue_size=1
        )

        # Servo on off
        self.servo_on_states = None
        rospy.Subscriber(
            "fullbody_controller/servo_on_off_real_interface/state",
            ServoOnOff,
            callback=self.servo_on_off_cb,
            queue_size=1,
        )
        self.init_finished = True

    def button_cb(self, msg):
        """
        When AtomS3 is ServoControlMode and single-click pressed,
        toggle servo control.
        """
        if self.mode == "ServoControlMode" and msg.data == 1:
            rospy.loginfo(
                "AtomS3 is ServoControlMode and single-click-pressed."
                + " Toggle servo control."
            )
            self.toggle_servo_on_off()

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
        if self.mode != "ServoControlMode":
            return
        # Send message on AtomS3 LCD
        if self.init_finished is False:
            sent_str = chr(PacketType.SERVO_CONTROL_MODE)
            sent_str += "Servo Control Mode\n\n"
            sent_str += "Wait for servo response"
            self.write(sent_str)
            return
        sent_str = chr(PacketType.SERVO_CONTROL_MODE)
        sent_str += "Servo control mode\n\nSingle Click:\nServo on off"
        self.write(sent_str)


if __name__ == "__main__":
    rospy.init_node("servo_control_mode")
    scm = ServoControlMode()
    rospy.spin()
