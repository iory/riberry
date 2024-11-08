#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, Float32
from std_msgs.msg import String

from riberry.i2c_base import I2CBase


class DisplayBatteryMode(I2CBase):
    def __init__(self, i2c_addr, lock_path="/tmp/i2c_display_battery_mode.lock"):
        super().__init__(i2c_addr)

        self.mode = None
        # rospy.Subscriber(
        #     "/i2cbutton_state", Int32, callback=self.button_cb, queue_size=1
        # )
        rospy.Subscriber("/i2c_mode", String, callback=self.mode_cb, queue_size=1)
        rospy.Subscriber("battery_voltage_status", Float32, callback=self.voltage_cb, queue_size=1)
        self.voltage = 0.0

        rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def voltage_cb(self, msg):
        """
        When AtomS3 is ServoControlMode and single-click pressed,
        toggle servo control.
        """
        if self.mode == "DisplayBatteryMode1":
            self.voltage = msg.data

    def mode_cb(self, msg):
        """
        Check AtomS3 mode.
        """
        self.mode = msg.data

    def timer_callback(self, event):
        # if self.mode != "ServoControlMode":
            # return
        # Send message on AtomS3 LCD
        # sent_str = "Servo control mode\n\nSingle Click:\nServo on off"
        # self.send_string(sent_str)
        if self.mode=="DisplayBatteryMode1":
            self.send_string(str(self.voltage))
        elif self.mode=="DisplayBatteryMode2":
            self.send_string("fuga")


if __name__ == "__main__":
    rospy.init_node("display_battery_mode")
    scm = DisplayBatteryMode(0x42)
    rospy.spin()
