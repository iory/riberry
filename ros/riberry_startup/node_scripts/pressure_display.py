#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import String


class PressureDisplay(object):
    def __init__(self):
        self.pub = rospy.Publisher('/atom_s3_additional_info',
                                   String, queue_size=10)
        self.pressures = {}
        for idx in range(38, 66):
            rospy.Subscriber('/kxr_fullbody_controller/pressure/{}'
                             .format(idx), Float32, self.read_pressure,
                             callback_args=idx)
            self.pressures[idx] = None
        rospy.Timer(rospy.Duration(1), self.timer_callback)

    def read_pressure(self, msg, idx):
        self.pressures[idx] = msg.data

    def timer_callback(self, event):
        text = 'pressure [kPa]\n'
        for idx, value in self.pressures.items():
            if value is None:
                continue
            else:
                text += '{}: {:.3f}\n'.format(idx, value)
        pressure_msg = String(data=text)
        self.pub.publish(pressure_msg)


if __name__ == '__main__':
    rospy.init_node('pressure_display')
    pd = PressureDisplay()
    rospy.spin()
