import rospy
from std_msgs.msg import String

from riberry.com.base import ComBase
from riberry.com.i2c_base import I2CBase
from riberry.com.uart_base import UARTBase


class Mode:
    def __init__(self):
        device = ComBase.identify_device()
        if device in ['m5stack-LLM', 'Linux', 'Darwin']:
            self.com = UARTBase()
        else:
            self.com = I2CBase(0x42)

        self.mode = None
        rospy.Subscriber("atom_s3_mode", String, callback=self.mode_cb, queue_size=1)

    def write(self, data):
        return self.com.write(data)

    def read(self):
        return self.com.read()

    def mode_cb(self, msg):
        self.mode = msg.data
