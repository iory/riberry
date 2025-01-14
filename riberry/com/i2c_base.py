import fcntl
import sys

from filelock import FileLock
from filelock import Timeout
from i2c_for_esp32 import WirePacker

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
        buffer_size = len(data) + 8
        packer = WirePacker(buffer_size=buffer_size)

        if isinstance(data, str):
            for s in data:
                try:
                    packer.write(ord(s))
                except ValueError as e:
                    print(f'[ERROR] {e} Invalid character {s}')
        elif isinstance(data, (bytes, bytearray)):
            for r in data:
                packer.write(r)
        else:
            raise TypeError(f'Unsupported data type: {type(data)}. Expected str or bytes.')

        packer.end()
        if packer.available():
            self.i2c_write(packer.buffer[: packer.available()])
