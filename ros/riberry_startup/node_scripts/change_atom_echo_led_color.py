#!/usr/bin/env python3


from i2c_for_esp32 import WirePacker
import rospy
from std_msgs.msg import ColorRGBA

from riberry.i2c_base import I2C

if __name__ == "__main__":
    rospy.init_node("change_atom_echo_led_color")

    i2c = I2C(0x41, bus=3)

    def callback(msg):
        r, g, b = int(msg.r), int(msg.g), int(msg.b)
        rospy.loginfo(
            f"Change Atom echo LED Color R:{r} G:{g} B:{b}"
        )
        input_string = chr(r) + chr(g) + chr(b)
        packer = WirePacker(buffer_size=len(input_string) + 8)
        for s in input_string:
            packer.write(ord(s))
        packer.end()
        if packer.available():
            i2c.write(packer.buffer[: packer.available()])

    sub = rospy.Subscriber("color_rgb", ColorRGBA, queue_size=1, callback=callback)
    rospy.loginfo('Change Atom echo led color node start.')
    rospy.spin()
