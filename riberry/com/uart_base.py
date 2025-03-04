from pySerialTransfer import pySerialTransfer as txfer
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

    def __init__(self, serial_port=None, baudrate=921600):
        self.baudrate = baudrate
        self.serial = None
        self._connect_serial(serial_port)
        super().__init__(self.device_name)

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
            self.serial = txfer.SerialTransfer(serial_port, baud=self.baudrate,
                                               restrict_ports=False)
            self.device_name = serial_port[len("/dev/"):]
            return True
        except serial.serialutil.SerialException:
            print("[uart_base] Serial connection failed.")
            return False

    def reset_input_buffer(self):
        try:
            self.serial.connection.reset_input_buffer()
        except serial.serialutil.PortNotOpenError:
            print("[uart_base] failed to reset input buffer")

    def _write(self, data, raw=False):
        try:
            if self.serial is None:
                print("[uart_base] Serial is not initialized. Try to connect serial.")
                self._connect_serial()
                return
            if isinstance(data, str):
                # The limit must be under Serial's receive buffer size
                # For AtomS3, about 160 is limit
                packet = str_to_byte_list(data, 150)
            elif isinstance(data, (bytes, bytearray)):
                packet = data
            elif isinstance(data, list):
                if all(isinstance(item, int) for item in data):
                    # If all elements are integers, treat as raw ASCII values
                    packet = data
                elif all(isinstance(item, str) and len(item) == 1 for item in data):
                    # If all elements are single-character strings, convert to ASCII values
                    data_str = ''.join(data)  # Combine list into a single string
                    # The limit must be under Serial's receive buffer size
                    # For AtomS3, about 160 is limit
                    packet = str_to_byte_list(data_str, 150)
                else:
                    raise ValueError('List must contain either all integers or all single-character strings.')
            else:
                raise TypeError(f'Unsupported data type: {type(data)}. Expected str or bytes.')
            if raw:
                self.serial.connection.write(packet)
            else:
                send_size = 0
                for p in packet:
                    send_size = self.serial.tx_obj(p, start_pos=send_size, val_type_override='B')
                self.serial.send(send_size)
        except OSError as e:
            print(f"[uart_base] {e}. Restart serial.")
            self._connect_serial()

    def write(self, data, raw=False):
        try:
            with self.lock:
                self._write(data, raw)
        except Exception as e:
            print("[uart_base] Error during write:", e)

    # read() must return bytes, not None
    def read(self):
        try:
            if self.serial is None:
                print("[uart_base] Serial is not initialized. Try to connect serial.")
                self._connect_serial()
                return b''
            available = self.serial.available()
            if available == 0:
                return b''
            received = self.serial.rx_obj(obj_type=list, list_format='B', obj_byte_size=available)
            return bytes(received)
        except Exception as e:
            print("[uart_base] Error during read:", e)
            self._connect_serial()
            return b''
