#!/usr/bin/env python3


import board
import busio
from i2c_for_esp32 import WirePacker
import rospy
from std_msgs.msg import ColorRGBA

if __name__ == "__main__":
    rospy.init_node("change_atom_echo_led_color")

    i2c = busio.I2C(board.SCL3, board.SDA3)
    i2c_addr = 0x41
    while not i2c.try_lock():
        pass

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
            i2c.writeto(i2c_addr, packer.buffer[: packer.available()])

    sub = rospy.Subscriber("color_rgb", ColorRGBA, queue_size=1, callback=callback)
    rospy.loginfo('Change Atom echo led color node start.')
    rospy.spin()
