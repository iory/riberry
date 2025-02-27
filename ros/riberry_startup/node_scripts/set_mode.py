#!/usr/bin/env python3

import rospy

from riberry.com.base import PacketType
from riberry.mode import Mode


class SetMode(Mode):
    def __init__(self):
        super().__init__()
        self.mode_names = self.get_mode_names_from_rosparam()
        rospy.Timer(rospy.Duration(1), self.timer_callback)

    def get_mode_names_from_rosparam(self):
        try:
            mode_name_list = rospy.get_param("~mode_names", "")
            return ",".join(mode_name_list)
        except KeyError:
            rospy.logwarn("rosparam 'mode_names' が見つかりませんでした。空のリストを設定します。")
            return ""

    def timer_callback(self, event):
        header = [PacketType.SELECTED_MODE]
        forceModebytes = (list (map(ord, self.mode_names)))
        rospy.loginfo_throttle(60, f"Mode names: {self.mode_names}")
        self.write(header + forceModebytes)


if __name__ == "__main__":
    rospy.init_node("set_mode")
    SetMode()
    rospy.spin()
