#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import String

from kxr_controller.kxr_interface import KXRROSRobotInterface
from kxr_controller.msg import PressureControl
from skrobot.model import RobotModel

from i2c_for_esp32 import WirePacker
from filelock import FileLock
from filelock import Timeout


class PressureControlMode(object):
    def __init__(self, i2c_addr):
        # Create robot model to control pressure
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
        
        # Pressure control
        self.pressure_control_state = {}
        rospy.Subscriber(
            "/kxr_fullbody_controller/pressure_control_interface/state",
            PressureControl,
            callback=self.pressure_control_cb, queue_size=1)

        # Read pressure
        self.pressures = {}
        for idx in range(38, 66):
            rospy.Subscriber('/kxr_fullbody_controller/pressure/{}'
                             .format(idx), Float32, self.read_pressure,
                             callback_args=idx)
            self.pressures[idx] = None
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
        When AtomS3 is Pressurecontrolmode and single-click pressed,
        toggle pressure control.
        """
        if self.mode == 'PressureControlMode' and msg.data == 1:
            rospy.loginfo(
                'AtomS3 is Pressurecontrolmode and single-click-pressed.'
                ' Toggle pressure control.')
            self.toggle_pressure_control()

    def mode_cb(self, msg):
        """
        Check AtomS3 mode.
        """
        self.mode = msg.data
            
    def pressure_control_cb(self, msg):
        self.pressure_control_state[f'{msg.board_idx}'] = msg

    def toggle_pressure_control(self):
        """
        Toggle release status
        """
        if self.ri is None:
            rospy.logwarn("KXRROSRobotInterface instance is not created.")
            return
        board_ids = list(self.pressure_control_state.keys())
        for idx in board_ids:
            state = self.pressure_control_state[f'{idx}']
            if state.start_pressure == 0 and state.stop_pressure == 0:
                start_pressure = -10
                stop_pressure = -30
            else:
                start_pressure = state.start_pressure
                stop_pressure = state.stop_pressure
            self.ri.send_pressure_control(
                board_idx=int(idx),
                start_pressure=start_pressure,
                stop_pressure=stop_pressure,
                release=(not state.release))

    def read_pressure(self, msg, idx):
        self.pressures[idx] = msg.data

    def timer_callback(self, event):
        if self.mode != 'PressureControlMode':
            return
        sent_str = 'pressure [kPa]\n'
        for idx, value in self.pressures.items():
            if value is None:
                continue
            else:
                sent_str += '{}: {:.3f}\n'.format(idx, value)
        sent_str += '\nSingle Click:\nVacuum on off'
        # Send message on AtomS3 LCD
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
    rospy.init_node('pressure_control_mode')
    pcm = PressureControlMode(0x42)
    rospy.spin()
