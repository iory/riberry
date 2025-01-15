import serial

from riberry.com.base import ComBase


class UARTBase(ComBase):

    def __init__(self, serial_port=None, baudrate=115200):
        super().__init__()
        if serial_port is None:
            device = self.identify_device()
            if device == 'm5stack-LLM':
                serial_port = '/dev/ttyS1'
            else:
                raise NotImplementedError(f"Not supported device {device}")
        self.serial = serial.Serial(
            port=serial_port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )

    def write(self, data):
        if isinstance(data, str):
            self.serial.write(list(map(ord, data)))
        elif isinstance(data, (bytes, bytearray)):
            self.serial.write(data)
        elif isinstance(data, list):
            if all(isinstance(item, int) for item in data):
                # If all elements are integers, treat as raw ASCII values
                self.serial.write(data)
            elif all(isinstance(item, str) and len(item) == 1 for item in data):
                # If all elements are single-character strings, convert to ASCII values
                data_str = ''.join(data)  # Combine list into a single string
                self.serial.write(list(map(ord, data_str)))
            else:
                raise ValueError('List must contain either all integers or all single-character strings.')
        else:
            raise TypeError(f'Unsupported data type: {type(data)}. Expected str or bytes.')

    def read(self):
        if self.serial.in_waiting:
            return self.serial.read(self.serial.in_waiting or 1)
        else:
            return b''
