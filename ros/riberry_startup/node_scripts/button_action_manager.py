#!/usr/bin/env python3

import threading

from kxr_controller.kxr_interface import KXRROSRobotInterface
from kxr_controller.msg import PressureControl
from kxr_controller.msg import ServoOnOff
import rospy
from skrobot.model import RobotModel
from std_msgs.msg import Int32


class ButtonActionManager(threading.Thread):
    """
    4 clicks to turn servo off, 5 clicks to turn servo on.
    The process may stop when creating a KXRROSRobotInterface instance.
    So the process was threaded so that it would not stop the main process.
    """

    def __init__(self):
        super().__init__(daemon=True)
        self.ri = None

        # Pressure control
        self.pressure_control_state = {}
        rospy.Subscriber(
            "/fullbody_controller/pressure_control_interface/state",
            PressureControl,
            callback=self.pressure_control_cb,
            queue_size=1,
        )
        # Servo on off
        self.servo_on_states = None
        rospy.Subscriber(
            "/fullbody_controller/servo_on_off_real_interface/state",
            ServoOnOff,
            callback=self.servo_on_off_cb,
            queue_size=1,
        )
        # Button
        rospy.Subscriber(
            "/atom_s3_button_state", Int32, callback=self.button_cb, queue_size=1
        )

        self.start()

    def pressure_control_cb(self, msg):
        self.pressure_control_state[f"{msg.board_idx}"] = msg

    def servo_on_off_cb(self, msg):
        self.servo_on_states = msg

    def button_cb(self, msg):
        """
        When AtomS3 is
        - pressed and holded, toggle pressure control.
        - pressed once, toggle servo on off.
        """
        state = msg.data
        if state == 11:
            rospy.loginfo("AtomS3 is pressed and holded. Toggle pressure control.")
            self.toggle_pressure_control()
        if state == 1:
            rospy.loginfo("AtomS3 is pressed once. Toggle servo on off.")
            self.toggle_servo_on_off()

    def toggle_pressure_control(self):
        """
        Toggle release status
        """
        if self.ri is None:
            rospy.logwarn("KXRROSRobotInterface instance is not created.")
            return
        board_ids = list(self.pressure_control_state.keys())
        for idx in board_ids:
            state = self.pressure_control_state[f"{idx}"]
            if state.start_pressure == 0 and state.stop_pressure == 0:
                start_pressure = -10
                stop_pressure = -30
            else:
                start_pressure = state.start_pressure
                stop_pressure = state.stop_pressure
            self.ri.send_pressure_control(
                board_idx=int(idx),
                start_pressure=start_pressure,
                stop_pressure=stop_pressure,
                release=(not state.release),
            )

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

    def run(self):
        """
        self.run() is executed when the self.start() is called.
        """
        robot_model = RobotModel()
        namespace = ""
        robot_model.load_urdf_from_robot_description(namespace + "/robot_description_viz")
        self.ri = KXRROSRobotInterface(
            robot_model, namespace=namespace, controller_timeout=60.0
        )


if __name__ == "__main__":
    """
    All AtomS3 clicking actions are integrated into this program
    to prevent different actions from being assigned to the same click.
    """
    rospy.init_node("button_action_manager")
    ButtonActionManager()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Finish handle_button_press")
