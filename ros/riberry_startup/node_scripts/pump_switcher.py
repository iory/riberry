#!/usr/bin/env python3

import board
import digitalio
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Empty


class PumpSwitcher:
    def __init__(self):
        rospy.loginfo("Start PumpSwitcher. Default is pump on.")
        # For pinout, see https://wiki.radxa.com/Zero/hardware/gpio
        # D19: GPIOH_4
        self.mosfet = digitalio.DigitalInOut(board.D19)
        self.mosfet.direction = digitalio.Direction.OUTPUT
        self.mosfet.value = False  # default: D19 is low
        self.prev_state = self.mosfet.value
        # ROS publisher
        self.state_pub = rospy.Publisher("pump_state", Bool, queue_size=10)
        # ROS subscribers
        rospy.sleep(1.0)
        rospy.Subscriber("pump_on", Empty, self.on_cb)
        rospy.Subscriber("pump_off", Empty, self.off_cb)
        # Timer to publish state at regular intervals
        self.timer = rospy.Timer(rospy.Duration(1), self.publish_state)

    def on_cb(self, msg):
        if self.prev_state is False:
            rospy.loginfo("Pump on")
        self.mosfet.value = True
        self.prev_state = self.mosfet.value

    def off_cb(self, msg):
        if self.prev_state is True:
            rospy.loginfo("Pump off")
        self.mosfet.value = False
        self.prev_state = self.mosfet.value

    def publish_state(self, event=None):
        rospy.logdebug(f"Publishing pump state: {self.mosfet.value}")
        self.state_pub.publish(self.mosfet.value)


if __name__ == "__main__":
    rospy.init_node("pump_switcher")
    PumpSwitcher()
    rospy.spin()
