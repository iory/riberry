#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import String

from riberry.battery import ChargeState
from riberry.com.base import PacketType
from riberry.mode import Mode


class DisplayBatteryGraphMode(Mode):
    def __init__(self):
        super().__init__()

        self.charge_str_to_num = {str(state): state.value
                                    for state in ChargeState}
        self.charge_status = None
        self.charge_current = 0
        self.display_duration = rospy.get_param("~display_duration", 3600)
        # Assume battery topic is 1Hz
        self.display_bins = 10
        self.battery_percentages = [0] * self.display_duration
        rospy.Subscriber("battery/remaining_battery", Float32,
                         callback=self.battery_cb, queue_size=1)
        rospy.Subscriber("battery/charge_status_string", String,
                         callback=self.status_cb, queue_size=1)
        rospy.Subscriber("battery/battery_charge_current", Float32,
                         callback=self.current_cb, queue_size=1)
        rospy.Timer(rospy.Duration(1), self.timer_callback)

    def mode_cb(self, msg):
        """
        Check AtomS3 mode.
        """
        previous_mode = self.mode
        super().mode_cb(msg)
        if self.mode == "DisplayBatteryGraphMode":
            if previous_mode != self.mode:
                rospy.loginfo('start display battery graph mode')
                self.send_data()

    def battery_cb(self, msg):
        for i in range(self.display_duration-1):
            self.battery_percentages[i] = self.battery_percentages[i+1]
        self.battery_percentages[-1] = msg.data

    def status_cb(self, msg):
        previous_status = self.charge_status
        self.charge_status = self.charge_str_to_num[msg.data]
        if self.mode == "DisplayBatteryGraphMode":
            if previous_status != self.charge_status:
                self.send_data()

    def current_cb(self, msg):
        self.charge_current = int(msg.data)

    def timer_callback(self, event):
        if self.mode != "DisplayBatteryGraphMode":
            return
        self.send_data()

    def send_data(self):
        sent_str = [chr(PacketType.DISPLAY_BATTERY_GRAPH_MODE)]
        sent_str += f'{self.charge_status},{self.charge_current},{self.display_duration},'
        for i in range(self.display_bins):
            percentage = self.battery_percentages[
                int((i+1)*self.display_duration/self.display_bins)-1]
            sent_str += f'{percentage}'
            if i != self.display_bins-1:
                sent_str += ','
        self.write(sent_str)


if __name__ == "__main__":
    rospy.init_node("display_battery_graph_mode")
    dbgm = DisplayBatteryGraphMode()
    rospy.spin()
