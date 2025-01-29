from enum import IntEnum
import os.path as osp
import platform


class PacketType(IntEnum):
    TEXT = 0x00
    JPEG = 0x01
    QR_CODE = 0x02
    FORCE_MODE = 0x03
    SELECTED_MODE = 0x04
    DISPLAY_BATTERY_GRAPH_MODE = 0x05
    SERVO_CONTROL_MODE = 0x06
    PRESSURE_CONTROL_MODE = 0x07
    TEACHING_MODE = 0x08
    DISPLAY_ODOM_MODE = 0x09
    BUTTON_STATE_REQUEST = 0x10
    GET_PAIRING_TYPE = 0x11
    PAIRING_IP_REQUEST = 0x12
    SET_IP_REQUEST = 0x13


class ComBase:

    def __init__(self):
        self.device_type = self.identify_device()

    def write(self, data):
        raise NotImplementedError('You should implement write function.')

    def reset_input_buffer(self):
        pass

    @staticmethod
    def identify_device():
        try:
            if osp.exists("/proc/cpuinfo"):
                with open("/proc/cpuinfo") as f:
                    cpuinfo = f.read()
                if "Raspberry Pi" in cpuinfo:
                    return "Raspberry Pi"
            if osp.exists("/proc/device-tree/model"):
                with open("/proc/device-tree/model") as f:
                    model = f.read().strip().replace("\x00", "")
                if "Radxa" in model or "ROCK Pi" in model\
                   or model == "Khadas VIM4"\
                   or model == "NVIDIA Jetson Xavier NX Developer Kit":
                    return model
            if osp.exists('/usr/local/m5stack/block-mount.sh'):
                return 'm5stack-LLM'
            if platform.system() == 'Linux':
                return 'Linux'
            elif platform.system()== 'Darwin':
                return 'Darwin'
            return "Unknown Device"
        except FileNotFoundError:
            return "Unknown Device"
