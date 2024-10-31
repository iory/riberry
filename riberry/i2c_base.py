import fcntl
import sys

from filelock import FileLock
from filelock import Timeout
from i2c_for_esp32 import WirePacker

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


class I2CBase:
    def __init__(self, i2c_addr):
        self.i2c_addr = i2c_addr
        self.device_type = self.identify_device()
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

    def send_string(self, sent_str):
        packer = WirePacker(buffer_size=len(sent_str) + 8)
        for s in sent_str:
            packer.write(ord(s))
        packer.end()
        if packer.available():
            self.i2c_write(packer.buffer[: packer.available()])

    def send_raw_bytes(self, raw_bytes):
        packer = WirePacker(buffer_size=len(raw_bytes) + 8)
        for r in raw_bytes:
            packer.write(r)
        packer.end()
        if packer.available():
            self.i2c_write(packer.buffer[: packer.available()])

    @staticmethod
    def identify_device():
        try:
            with open("/proc/cpuinfo") as f:
                cpuinfo = f.read()
            if "Raspberry Pi" in cpuinfo:
                return "Raspberry Pi"
            with open("/proc/device-tree/model") as f:
                model = f.read().strip().replace("\x00", "")
            if "Radxa" in model or "ROCK Pi" in model\
               or model == "Khadas VIM4"\
               or model == "NVIDIA Jetson Xavier NX Developer Kit":
                return model
            return "Unknown Device"
        except FileNotFoundError:
            return "Unknown Device"
