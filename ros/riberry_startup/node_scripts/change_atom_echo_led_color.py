#!/usr/bin/env python3


from filelock import FileLock
from filelock import Timeout
from i2c_for_esp32 import WirePacker
import rospy
from std_msgs.msg import ColorRGBA

from riberry.i2c_base import I2C

if __name__ == "__main__":
    rospy.init_node("change_atom_echo_led_color")

    bus = 3
    i2c = I2C(0x41, bus=bus)
    lock_path = f"/tmp/i2c-{bus}.lock"
    lock = FileLock(lock_path, timeout=10)

    def callback(msg):
        r, g, b = int(msg.r), int(msg.g), int(msg.b)
        r = min(max(0, r), 255)
        g = min(max(0, g), 255)
        b = min(max(0, b), 255)
        rospy.loginfo(
            f"Change Atom echo LED Color R:{r} G:{g} B:{b}"
        )
        input_string = chr(r) + chr(g) + chr(b)
        packer = WirePacker(buffer_size=len(input_string) + 8)
        for s in input_string:
            packer.write(ord(s))
        packer.end()
        if packer.available():
            try:
                lock.acquire()
            except Timeout as e:
                rospy.logerr(str(e))
                return
            try:
                i2c.write(packer.buffer[: packer.available()])
            except OSError as e:
                rospy.logerr(str(e))
            except TimeoutError as e:
                rospy.logerr(f"I2C Write error {e}")
            finally:
                try:
                    lock.release()
                except Timeout as e:
                    rospy.logerr(str(e))


    sub = rospy.Subscriber("color_rgb", ColorRGBA, queue_size=1, callback=callback)
    rospy.loginfo('Change Atom echo led color node start.')
    rospy.spin()
