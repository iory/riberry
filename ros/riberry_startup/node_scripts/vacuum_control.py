#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from std_msgs.msg import Int16


class VacuumControl:
    def __init__(self, vacuum_threshold=80):
        # Use Radxa's ADC value as vacuum pressure value
        self.vacuum_threshold = vacuum_threshold
        rospy.loginfo(f"Differential pressure threshold: {self.vacuum_threshold}")
        # Use pump and pressure sensor
        self.pub_on = rospy.Publisher("pump_on", Empty, queue_size=1)
        self.pub_off = rospy.Publisher("pump_off", Empty, queue_size=1)
        rospy.Subscriber("vacuum_pressure", Int16, self.pressure_cb)
        # Control vacuum state
        self.vacuum = False
        self.state_pub = rospy.Publisher("vacuum_control_state", Bool, queue_size=10)
        rospy.Subscriber("vacuum_on", Empty, self.on_cb)
        rospy.Subscriber("vacuum_off", Empty, self.off_cb)
        rospy.Subscriber("vacuum_toggle", Empty, self.toggle_cb)
        # Calibate pressure
        self.pressure_samples = []
        self.atm_pressure = None
        rospy.Subscriber("calibrate_pressure", Empty, self.calibrate)
        self.timer = rospy.Timer(rospy.Duration(1), self.publish_state)

    def calibrate(self, msg):
        self.pressure_samples = []
        self.atm_pressure = None

    def on_cb(self, msg):
        rospy.logwarn("Start vacuuming")
        self.vacuum = True

    def off_cb(self, msg):
        rospy.logwarn("Stop vacuuming")
        self.vacuum = False
        self.pub_off.publish(Empty())

    def toggle_cb(self, msg):
        rospy.logwarn("Toggle vacuum state")
        if self.vacuum is False:
            self.vacuum = True
            self.pub_on.publish(Empty())
        elif self.vacuum is True:
            self.vacuum = False
            self.pub_off.publish(Empty())

    def pressure_cb(self, msg):
        vacuum_pressure = msg.data
        # Calibrate pressure when startup or self.calibrate() is called
        if self.atm_pressure is None:
            required_samples = 5
            if len(self.pressure_samples) < required_samples:
                self.pressure_samples.append(vacuum_pressure)
                rospy.loginfo(
                    f"Collecting pressure data for calibration. {len(self.pressure_samples)}/{required_samples}"
                )
            else:
                self.atm_pressure = sum(self.pressure_samples) / float(
                    len(self.pressure_samples)
                )
                rospy.loginfo(
                    f"Calibration finished. Atmosphere pressure is {self.atm_pressure:.1f}"
                )
            return
        differential_pressure = self.atm_pressure - vacuum_pressure
        rospy.logdebug(f"Differential pressure: {differential_pressure:.1f}")
        # Control vacuum state
        if self.vacuum is True:
            if differential_pressure < self.vacuum_threshold:
                rospy.logwarn("Pump ON")
                self.pub_on.publish(Empty())
            else:
                self.pub_off.publish(Empty())

    def publish_state(self, event=None):
        rospy.logdebug(f"Publishing vacuum control state: {self.vacuum}")
        self.state_pub.publish(self.vacuum)


if __name__ == "__main__":
    rospy.init_node("vacuum_control")
    vc = VacuumControl()
    rospy.spin()
