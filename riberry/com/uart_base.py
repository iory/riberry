from filelock import FileLock
from filelock import Timeout
import serial
import serial.tools.list_ports

from riberry.com.base import ComBase
from riberry.com.base import str_to_byte_list


def usb_devices():
    """Find the port to which the ESP32 is connected

    Returns usb_ports[str]: e.g. /dev/ttyACM0
    """
    usb_ports = []
    available_tty_types = ["USB", "ACM"]
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "USB" in port.description:
            usb_ports.append(port.device)
            continue
        if any(tty_type in port.device for tty_type in available_tty_types):
            usb_ports.append(port.device)
            continue
    return usb_ports


class UARTBase(ComBase):

    def __init__(self, serial_port=None, baudrate=115200):
        super().__init__()
        self.baudrate = baudrate
        self.serial = None
        self._connect_serial(serial_port)

    def _connect_serial(self, serial_port=None):
        """Reconnect if port disappear"""
        try:
            if self.serial is not None:
                self.serial.close()
            if serial_port is None:
                device = self.identify_device()
                if device == 'm5stack-LLM':
                    serial_port = '/dev/ttyS1'
                elif device in ['Linux', 'Darwin']:
                    usb_ports = usb_devices()
                    if len(usb_ports) == 0:
                        print('Cannot find USB devices')
                        return False
                    serial_port = usb_ports[0]
                    print(serial_port)
                else:
                    raise NotImplementedError(f"Not supported device {device}")
            self.serial = serial.Serial(
                port=serial_port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1,  # read timeout
                write_timeout=1
            )
            device_name = serial_port[len("/dev/"):]
            self.lock = FileLock(f"/tmp/{device_name}_lock", timeout=10)
            return True
        except serial.serialutil.SerialException:
            print("[uart_base] Serial connection failed.")
            return False

    def reset_input_buffer(self):
        try:
            self.serial.reset_input_buffer()
        except serial.serialutil.PortNotOpenError:
            print("[uart_base] failed to reset input buffer")

    def write(self, data):
        try:
            self.lock.acquire()
        except Timeout as e:
            print(e)
            return
        try:
            if self.serial is None:
                print("[uart_base] Serial is not initialized. Try to connect serial.")
                self._connect_serial()
                return
            if isinstance(data, str):
                # The limit must be under Serial's receive buffer size
                # For AtomS3, about 160 is limit
                self.serial.write(str_to_byte_list(data, 150))
            elif isinstance(data, (bytes, bytearray)):
                self.serial.write(data)
            elif isinstance(data, list):
                if all(isinstance(item, int) for item in data):
                    # If all elements are integers, treat as raw ASCII values
                    self.serial.write(data)
                elif all(isinstance(item, str) and len(item) == 1 for item in data):
                    # If all elements are single-character strings, convert to ASCII values
                    data_str = ''.join(data)  # Combine list into a single string
                    # The limit must be under Serial's receive buffer size
                    # For AtomS3, about 160 is limit
                    self.serial.write(str_to_byte_list(data_str, 150))
                else:
                    raise ValueError('List must contain either all integers or all single-character strings.')
            else:
                raise TypeError(f'Unsupported data type: {type(data)}. Expected str or bytes.')
        except OSError as e:
            print(f"[uart_base] {e}. Restart serial.")
            self._connect_serial()
        finally:
            try:
                self.lock.release()
            except Timeout as e:
                print(e)

    def read(self):
        try:
            if self.serial is None:
                print("[uart_base] Serial is not initialized. Try to connect serial.")
                self._connect_serial()
                return
            if self.serial.in_waiting:
                return self.serial.read(self.serial.in_waiting or 1)
            else:
                return b''
        except (OSError, TypeError):
            print("[uart_base] Serial read failed. Restart serial.")
            try:
                self._connect_serial()
            except serial.serialutil.SerialException:
                print("[uart_base] Serial reconnection failed.")
