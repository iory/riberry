#!/usr/bin/env python3

from kxr_controller.kxr_interface import KXRROSRobotInterface
from kxr_controller.msg import PressureControl
import rospy
from skrobot.model import RobotModel
from skrobot.utils.urdf import no_mesh_load_mode
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import String

from riberry.com.base import PacketType
from riberry.com.i2c_base import I2CBase
from riberry.select_list import SelectList


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
            "atom_s3_button_state", Int32, callback=self.button_cb, queue_size=1
        )
        rospy.Subscriber("atom_s3_mode", String, callback=self.mode_cb, queue_size=1)

        # Pressure control
        self.pressure_control_state = {}
        rospy.Subscriber(
            "fullbody_controller/pressure_control_interface/state",
            PressureControl,
            callback=self.pressure_control_cb,
            queue_size=1,
        )
        self.air_work_list = SelectList()
        self.delimiter = ":"

        # Read pressure
        self.pressures = {}
        for idx in range(38, 66):
            rospy.Subscriber(
                f"fullbody_controller/pressure/{idx}",
                Float32,
                self.read_pressure,
                callback_args=idx,
            )
            self.pressures[idx] = None
        rospy.Timer(rospy.Duration(1), self.timer_callback)

    def button_cb(self, msg):
        """
        When AtomS3 is Pressurecontrolmode and single-click pressed,
        toggle pressure control.
        """
        if self.mode != "PressureControlMode":
            return
        if msg.data == 1:
            self.air_work_list.increment_index()
            self.send_string()
        if msg.data == 2:
            selected = self.air_work_list.selected_option()
            idx = int(selected.split(self.delimiter)[0])
            rospy.loginfo(
                "AtomS3 is Pressurecontrolmode and double-click-pressed."
                + f" Toggle ID {idx} pressure control."
            )
            self.toggle_pressure_control(idx)
            self.send_string()

    def mode_cb(self, msg):
        """
        Check AtomS3 mode.
        """
        self.mode = msg.data

    def pressure_control_cb(self, msg):
        self.pressure_control_state[f"{msg.board_idx}"] = msg

    def toggle_pressure_control(self, idx):
        """
        Toggle release status
        """
        if self.ri is None:
            rospy.logwarn("KXRROSRobotInterface instance is not created.")
            return
        state = self.pressure_control_state[f"{idx}"]
        # toggle release state
        if state.release_duration == 0:
            release_duration = 2  # release air
        elif state.release_duration > 0:
            release_duration = 0  # start air work
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

    def read_pressure(self, msg, idx):
        self.pressures[idx] = msg.data

    def timer_callback(self, event):
        self.send_string()

    def send_string(self):
        if self.mode != "PressureControlMode":
            return
        sent_str = chr(PacketType.PRESSURE_CONTROL_MODE)
        sent_str += "pressure [kPa]\n\n"
        air_work_index = self.air_work_list.get_index()
        self.air_work_list.remove_all_options()
        for idx, value in self.pressures.items():
            if value is None:
                continue
            else:
                air_work_str = f"{idx}{self.delimiter} {value:.1f}"
                if f"{idx}" in self.pressure_control_state:
                    release_duration = self.pressure_control_state[f"{idx}"].release_duration
                    if release_duration == 0:
                        air_work_str += "\x1b[32m ON\x1b[39m"
                    elif release_duration > 0:
                        air_work_str += "\x1b[31m OFF\x1b[39m"
                if air_work_str.count(self.delimiter) != 1:
                    rospy.logerr(f"The number of delimiter {self.delimiter} must be 1")
                self.air_work_list.add_option(air_work_str)
        self.air_work_list.set_index(air_work_index)
        sent_str += self.air_work_list.string_options(3)
        sent_str += "\n1 tap: Next"
        sent_str += "\n2 tap: Toggle"
        # Send message on AtomS3 LCD
        self.write(sent_str)


if __name__ == "__main__":
    rospy.init_node("pressure_control_mode")
    pcm = PressureControlMode(0x42)
    rospy.spin()
