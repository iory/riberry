#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import String

from kxr_controller.kxr_interface import KXRROSRobotInterface
from kxr_controller.msg import ServoOnOff
from skrobot.model import RobotModel

from i2c_for_esp32 import WirePacker
from filelock import FileLock
from filelock import Timeout


class ServoControlMode(object):
    def __init__(self, i2c_addr):
        # Create robot model to control servo
        robot_model = RobotModel()
        namespace = ""
        robot_model.load_urdf_from_robot_description(
            namespace + '/robot_description_viz')
        self.ri = KXRROSRobotInterface(  # NOQA
            robot_model, namespace=namespace, controller_timeout=60.0)

        # Button and mode callback
        self.mode = None
        rospy.Subscriber(
            "/atom_s3_button_state", Int32,
            callback=self.button_cb, queue_size=1)
        rospy.Subscriber(
            "/atom_s3_mode", String,
            callback=self.mode_cb, queue_size=1)

        # Servo on off
        self.servo_on_states = None
        rospy.Subscriber(
            "/kxr_fullbody_controller/servo_on_off_real_interface/state",
            ServoOnOff, callback=self.servo_on_off_cb, queue_size=1)

        rospy.Timer(rospy.Duration(0.1), self.timer_callback)

        # I2C
        self.i2c_addr = i2c_addr
        self.device_type = identify_device()
        if self.device_type == 'Raspberry Pi':
            import board
            import busio
            self.i2c = busio.I2C(board.SCL, board.SDA)
            bus_number = 1
        elif self.device_type == 'Radxa Zero':
            import board
            import busio
            self.i2c = busio.I2C(board.SCL1, board.SDA1)
            bus_number = 3
        elif self.device_type == 'Khadas VIM4':
            self.i2c = i2c()
            bus_number = None
        else:
            raise ValueError('Unknown device {}'.format(
                self.device_type))
        self.lock = FileLock(lock_path, timeout=10)

    # TODO: Create base class to use I2C
    def i2c_write(self, packet):
        try:
            self.lock.acquire()
        except Timeout as e:
            print(e)
            return
        try:
            self.i2c.writeto(self.i2c_addr, packet)
        except OSError as e:
            print(e)
        except TimeoutError as e:
            print('I2C Write error {}'.format(e))
        try:
            self.lock.release()
        except Timeout as e:
            print(e)
            return

    def button_cb(self, msg):
        """
        When AtomS3 is ServoControlMode and single-click pressed,
        toggle servo control.
        """
        if self.mode == 'ServoControlMode' and msg.data == 1:
            rospy.loginfo(
                'AtomS3 is ServoControlMode and single-click-pressed.'
                ' Toggle servo control.')
            self.toggle_servo_on_off()

    def mode_cb(self, msg):
        """
        Check AtomS3 mode.
        """
        self.mode = msg.data
            
    def servo_on_off_cb(self, msg):
        self.servo_on_states = msg

    def toggle_servo_on_off(self):
        """
        If one of the servos is on, turn the entire servo off.
        If all of the servos is off, turn the entire servo on.
        """
        if self.ri is None:
            rospy.logwarn("KXRROSRobotInterface instance is not created.")
            return
        if self.servo_on_states is None:
            return
        servo_on_states = self.servo_on_states.servo_on_states
        if any(servo_on_states) is True:
            self.ri.servo_off()
        else:
            self.ri.servo_on()

    def timer_callback(self, event):
        if self.mode != 'ServoControlMode':
            return
        # Send message on AtomS3 LCD
        sent_str = 'Servo control mode\n\nSingle Click:\nServo on off'
        packer = WirePacker(buffer_size=len(sent_str) + 8)
        for s in sent_str:
            packer.write(ord(s))
        packer.end()
        if packer.available():
            self.i2c_write(packer.buffer[:packer.available()])

lock_path = '/tmp/i2c-1.lock'

def identify_device():
    try:
        with open('/proc/cpuinfo', 'r') as f:
            cpuinfo = f.read()

        if 'Raspberry Pi' in cpuinfo:
            return 'Raspberry Pi'

        with open('/proc/device-tree/model', 'r') as f:
            model = f.read().strip()

        # remove null character
        model = model.replace('\x00', '')

        if 'Radxa' in model or 'ROCK Pi' in model or model in 'Khadas VIM4':
            return model

        return 'Unknown Device'
    except FileNotFoundError:
        return 'Unknown Device'


if __name__ == '__main__':
    rospy.init_node('servo_control_mode')
    scm = ServoControlMode(0x42)
    rospy.spin()
