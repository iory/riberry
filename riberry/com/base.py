from contextlib import contextmanager
from enum import IntEnum
import os.path as osp
import platform

from filelock import FileLock
from filelock import Timeout


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
    SPEECH_TO_TEXT_MODE = 0x14
    GET_ADDITIONAL_REQUEST = 0x15

    CORE_DUMP_DATA_REQUEST = 0xFC
    FIRMWARE_VERSION_REQUEST = 0xFD
    FIRMWARE_UPDATE_MODE = 0xFF

class ComBase:

    def __init__(self, device_name):
        self.device_type = self.identify_device()
        self._is_context_locked = False
        self.lock = FileLock(f"/tmp/{device_name}.lock", timeout=10)

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

    @contextmanager
    def lock_context(self):
        try:
            if self.lock is None:
                raise AttributeError("Lock is not initialized")
            if not self._is_context_locked:
                self.lock.acquire()
                self._is_context_locked = True
            yield
        except Timeout as e:
            print(f"[{self.__class__.__name__}] Lock acquire timeout: {e}")
        finally:
            if self._is_context_locked:
                try:
                    self.lock.release()
                    self._is_context_locked = False
                except Timeout as e:
                    print(f"[{self.__class__.__name__}] Lock release timeout: {e}")


def truncate_byte_list(byte_list, limit):
    truncated_list = byte_list[:limit]
    try:
        byte_data = bytes(truncated_list)
        byte_data.decode('utf-8')
        return truncated_list
    except UnicodeDecodeError:
        for i in range(len(truncated_list) - 1, 0, -1):
            try:
                bytes(truncated_list[:i]).decode('utf-8')
                return truncated_list[:i]
            except UnicodeDecodeError:
                continue
        return []


def str_to_byte_list(str, limit):
    """Convert String into byte list because packer.write() and serial.write() requires 0~255 value.

    Example
    Input: 'aあ' ('a' is [97] and 'あ' is [227, 129, 130] in unicode)
    Output: [97, 227, 129, 130]
    """
    nested_byte_list = [list(x.encode('utf-8')) for x in str]
    byte_list = [item for sublist in nested_byte_list for item in sublist]
    if len(byte_list) > limit:
        print(
            f"Truncate string because input size {len(byte_list)} is over {limit} bytes")
        byte_list = truncate_byte_list(byte_list, limit)
        print(len(byte_list))
    return byte_list
