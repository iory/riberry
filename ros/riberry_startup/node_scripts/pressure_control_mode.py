#!/usr/bin/env python3

from kxr_controller.kxr_interface import KXRROSRobotInterface
from kxr_controller.msg import PressureControl
import rospy
from skrobot.model import RobotModel
from skrobot.utils.urdf import no_mesh_load_mode
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import String

from riberry.i2c_base import I2CBase
from riberry.i2c_base import PacketType


class PressureControlMode(I2CBase):
    def __init__(self, i2c_addr):
        super().__init__(i2c_addr)
        # Create robot model to control pressure
        robot_model = RobotModel()
        namespace = ""
        with no_mesh_load_mode():
            robot_model.load_urdf_from_robot_description(
                namespace + "/robot_description_viz")
        self.ri = KXRROSRobotInterface(
            robot_model, namespace=namespace, controller_timeout=60.0
        )

        # Button and mode callback
        self.mode = None
        rospy.Subscriber(
            "/atom_s3_button_state", Int32, callback=self.button_cb, queue_size=1
        )
        rospy.Subscriber("/atom_s3_mode", String, callback=self.mode_cb, queue_size=1)

        # Pressure control
        self.pressure_control_state = {}
        rospy.Subscriber(
            "/fullbody_controller/pressure_control_interface/state",
            PressureControl,
            callback=self.pressure_control_cb,
            queue_size=1,
        )
        self.release_duration = {}

        # Read pressure
        self.board_ids = []
        self.pressures = {}
        for idx in range(38, 66):
            rospy.Subscriber(
                f"/fullbody_controller/pressure/{idx}",
                Float32,
                self.read_pressure,
                callback_args=idx,
            )
            self.pressures[idx] = None
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def button_cb(self, msg):
        """
        When AtomS3 is Pressurecontrolmode and single-click pressed,
        toggle pressure control.
        """
        if self.mode == "PressureControlMode" and msg.data == 1:
            rospy.loginfo(
                "AtomS3 is Pressurecontrolmode and single-click-pressed."
                + " Toggle pressure control."
            )
            self.toggle_pressure_control()

    def mode_cb(self, msg):
        """
        Check AtomS3 mode.
        """
        self.mode = msg.data

    def pressure_control_cb(self, msg):
        self.pressure_control_state[f"{msg.board_idx}"] = msg
        self.board_ids = list(self.pressure_control_state.keys())
        for idx in self.board_ids:
            self.release_duration[f"{idx}"] = msg.release_duration

    def toggle_pressure_control(self):
        """
        Toggle release status
        """
        if self.ri is None:
            rospy.logwarn("KXRROSRobotInterface instance is not created.")
            return
        for idx in self.board_ids:
            state = self.pressure_control_state[f"{idx}"]
            # toggle release state
            if state.release_duration == 0:
                release_duration = 2  # release air
            elif state.release_duration > 0:
                release_duration = 0  # close air
            # Set pressures
            if state.trigger_pressure == 0 and state.target_pressure == 0:
                trigger_pressure = -10
                target_pressure = -30
            else:
                trigger_pressure = state.trigger_pressure
                target_pressure = state.target_pressure
            # Send goal
            self.ri.send_pressure_control(
                board_idx=int(idx),
                trigger_pressure=trigger_pressure,
                target_pressure=target_pressure,
                release_duration=release_duration
            )
            rospy.sleep(0.1)  # Send multiple goals at time intervals.

    def read_pressure(self, msg, idx):
        self.pressures[idx] = msg.data

    def timer_callback(self, event):
        if self.mode != "PressureControlMode":
            return
        sent_str = chr(PacketType.PRESSURE_CONTROL_MODE)
        sent_str += "pressure [kPa]\n"
        for idx, value in self.pressures.items():
            if value is None:
                continue
            else:
                sent_str += f"{idx}: {value:.1f}"
                if f"{idx}" in self.release_duration:
                    if self.release_duration[f"{idx}"] == 0:
                        sent_str += "\x1b[32m ON\x1b[39m"
                    elif self.release_duration[f"{idx}"] > 0:
                        sent_str += "\x1b[31m OFF\x1b[39m"
                sent_str += "\n"
        sent_str += "\nSingle Click:\nVacuum on off"
        # Send message on AtomS3 LCD
        self.send_string(sent_str)


if __name__ == "__main__":
    rospy.init_node("pressure_control_mode")
    pcm = PressureControlMode(0x42)
    rospy.spin()
