#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import String

from riberry.i2c_base import I2CBase


class DisplayBatteryGraphMode(I2CBase):
    def __init__(self, i2c_addr,
                 lock_path="/tmp/i2c_display_battery_graph_mode.lock"):
        super().__init__(i2c_addr)

        self.mode = None
        self.display_duration = rospy.get_param("~display_duration", 3600)
        # Assume battery topic is 1Hz
        self.display_bins = 10
        self.battery_percentages = [0] * self.display_duration
        rospy.Subscriber("/atom_s3_mode", String,
                         callback=self.mode_cb, queue_size=1)
        rospy.Subscriber("/battery/remaining_battery", Float32,
                         callback=self.battery_cb, queue_size=1)
        rospy.Timer(rospy.Duration(10), self.timer_callback)

    def mode_cb(self, msg):
        """
        Check AtomS3 mode.
        """
        self.mode = msg.data

    def battery_cb(self, msg):
        for i in range(self.display_duration-1):
            self.battery_percentages[i] = self.battery_percentages[i+1]
        self.battery_percentages[-1] = msg.data

    def timer_callback(self, event):
        if self.mode != "DisplayBatteryGraphMode":
            return
        sent_str = f'{self.display_duration},'
        for i in range(self.display_bins):
            percentage = self.battery_percentages[
                int((i+1)*self.display_duration/self.display_bins)-1]
            sent_str += f'{percentage}'
            if i != self.display_bins-1:
                sent_str += ','
        self.send_string(sent_str)


if __name__ == "__main__":
    rospy.init_node("display_battery_graph_mode")
    dbgm = DisplayBatteryGraphMode(0x42)
    rospy.spin()
