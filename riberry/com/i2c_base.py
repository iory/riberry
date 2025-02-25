import fcntl
import sys

from filelock import FileLock
from filelock import Timeout
from i2c_for_esp32 import WirePacker
from i2c_for_esp32 import WireUnpacker

from riberry.com.base import ComBase

if sys.hexversion < 0x03000000:

    def _b(x):
        return x
else:

    def _b(x):
        return x.encode("latin-1")


class I2C:

    def __init__(self, device=0x42, bus=5):
        self.bus = bus
        self.fr = open("/dev/i2c-" + str(bus), "rb", buffering=0)
        self.fw = open("/dev/i2c-" + str(bus), "wb", buffering=0)
        # set device address
        I2C_SLAVE = 0x0703
        fcntl.ioctl(self.fr, I2C_SLAVE, device)
        fcntl.ioctl(self.fw, I2C_SLAVE, device)

    def write(self, data):
        if type(data) is list:
            data = bytearray(data)
        elif type(data) is str:
            data = _b(data)
        self.fw.write(data)

    def read(self, count):
        return self.fr.read(count)

    def close(self):
        self.fw.close()
        self.fr.close()


class I2CBase(ComBase):

    def __init__(self, i2c_addr):
        super().__init__()
        self.i2c_addr = i2c_addr
        self.setup_i2c()
        lock_path = f"/tmp/i2c-{self.i2c.bus}.lock"
        self.lock = FileLock(lock_path, timeout=10)

    def setup_i2c(self):
        if self.device_type == "Raspberry Pi":
            self.i2c = I2C(self.i2c_addr, bus=1)
        elif self.device_type == "Radxa Zero":
            self.i2c = I2C(self.i2c_addr, bus=1)
        elif self.device_type == "Khadas VIM4":
            self.i2c = I2C(self.i2c_addr, bus=5)
        elif self.device_type == "NVIDIA Jetson Xavier NX Developer Kit":
            self.i2c = I2C(self.i2c_addr, bus=8)
        else:
            raise ValueError(f"Unknown device {self.device_type}")

    def i2c_write(self, packet):
        try:
            self.lock.acquire()
        except Timeout as e:
            print(e)
            return
        try:
            self.i2c.write(packet)
        except OSError as e:
            print(e)
        except TimeoutError as e:
            print(f"I2C Write error {e}")
        finally:
            try:
                self.lock.release()
            except Timeout as e:
                print(e)

    def write(self, data):
        # Allocate a buffer size of 4 times the length of the string
        # to allow Unicode (4-byte characters) as well as ASCII charactors
        buffer_size = len(data) * 4 + 2
        packer = WirePacker(buffer_size=buffer_size)

        def str_to_byte_list(str):
            """Convert String into byte list because packer.write() requires 0~255 value.

            Example
            Input: 'aあ' ('a' is [97] and 'あ' is [227, 129, 130] in unicode)
            Output: [97, 227, 129, 130]
            """
            nested_byte_list = [list(x.encode('utf-8')) for x in str]
            byte_list = [item for sublist in nested_byte_list for item in sublist]
            return byte_list

        if isinstance(data, str):
            for s in str_to_byte_list(data):
                try:
                    packer.write(s)
                except ValueError as e:
                    print(f'[ERROR] {e} Invalid character {s}')
        elif isinstance(data, (bytes, bytearray)):
            for r in data:
                packer.write(r)
        elif isinstance(data, list):
            if all(isinstance(item, int) for item in data):
                # If all elements are integers, treat as raw ASCII values
                for r in data:
                    packer.write(r)
            elif all(isinstance(item, str) and len(item) == 1 for item in data):

                # If all elements are single-character strings, convert to ASCII values
                data_str = ''.join(data)
                for r in str_to_byte_list(data_str):
                    packer.write(r)
            else:
                raise ValueError('List must contain either all integers or all single-character strings.')
        else:
            raise TypeError(f'Unsupported data type: {type(data)}. Expected str or bytes.')

        packer.end()
        if packer.available():
            self.i2c_write(packer.buffer[: packer.available()])

    def read(self):
        packet = self.i2c.read(100)
        unpacker = WireUnpacker(buffer_size=100)
        unpacker.reset()
        for x in packet:
            unpacker.write(x)
        return bytes(unpacker.buffer[:unpacker.payloadLength])
