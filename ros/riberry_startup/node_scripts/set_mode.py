#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

from riberry.com.base import PacketType
from riberry.mode import Mode
from riberry.mode_type import string_to_mode_type


class SetMode(Mode):
    def __init__(self):
        super().__init__()
        self.mode_names = self.get_mode_names_from_rosparam()
        self.last_send_time = rospy.Time.now()
        rospy.Subscriber(
            "atom_s3_selected_modes", String, callback=self.cb, queue_size=1)

    def get_mode_names_from_rosparam(self):
        try:
            mode_name_list = rospy.get_param("~mode_names", "")
            return ",".join(mode_name_list)
        except KeyError:
            rospy.logwarn("rosparam 'mode_names' is not found.")
            return

    def cb(self, msg):
        if self.mode_names != msg.data:
            current_time = rospy.Time.now()
            time_since_last_send = (current_time - self.last_send_time).to_sec()
            if time_since_last_send >= 3.0:
                self.send_selected_modes()
                self.last_send_time = current_time

    def send_selected_modes(self):
        rospy.loginfo(f"Change selected mode: {self.mode_names}")
        header = [PacketType.SELECTED_MODE]
        mode_byte_list = [
            string_to_mode_type(mode_name).value for mode_name in self.mode_names.split(",")]
        msg_size = len(mode_byte_list) + 1
        packet = header + [msg_size] + mode_byte_list
        self.write(packet)


if __name__ == "__main__":
    rospy.init_node("set_mode")
    SetMode()
    rospy.spin()
