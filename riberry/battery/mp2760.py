import threading
import time

import smbus2

from riberry.battery.common import majority_vote


class MP2760BatteryMonitor(threading.Thread):

    REG27H = 0x27  # Battery Charge Current
    REG26H = 0x26  # System Voltage
    REG24H = 0x24  # Input Current
    REG25H = 0x25  # Battery Voltage

    def __init__(self, bus_number=3, device_address=0x5c, alpha=0.9,
                 percentage_threshold=20,
                 history_size=10, debug=True):
        self.bus = smbus2.SMBus(bus_number)
        super().__init__()
        self.debug = debug
        self.bus_number = bus_number
        self.device_address = device_address
        self.alpha = alpha
        self.percentage_threshold = percentage_threshold
        self.history_size = history_size

        self.filtered_percentage = 0
        self.percentage_history = []
        self.charging_history = []

        self.bus = smbus2.SMBus(self.bus_number)
        self.lock = threading.Lock()
        self.running = True

    def __del__(self):
        print("[MP2760BatteryMonitor] Object is being deleted, cleaning up...")
        self.stop()
        self.set_adc_continuous_mode(False)
        self.bus.close()

    def is_outlier(self, current, history, threshold):
        if not history:
            return False
        ratio = sum(abs(current - h) > threshold
                    for h in history) / len(history)
        return ratio > 0.4

    def update_history(self, value, history):
        history.append(value)
        if len(history) > self.history_size:
            history.pop(0)

    def set_adc_continuous_mode(self, set_bit=True):
        try:
            word = self.bus.read_word_data(
                self.device_address,
                0x0e)
        except Exception as e:
            print('[Battery Monitor] Error reading from I2C: {}'.format(e))
            return
        current_bit = (word >> 7) & 1
        if current_bit == set_bit:
            print(
                '[Battery Monitor] 7bit is already set to the desired value. ',
                'No action needed.')
            return
        if set_bit:
            print('[Battery Monitor] Set ADC_CONV to Continuous')
            set_adc_word = word | (1 << 7)
        else:
            print('[Battery Monitor] Set ADC_CONV to One-shot conversion')
            set_adc_word = word & ~(1 << 7)
        try:
            self.bus.write_word_data(
                self.device_address,
                0x0e, set_adc_word)
        except Exception as e:
            print('[Battery Monitor] Error writing I2C: {}'.format(e))
            return

    def calculate_lipo_percentage(self, voltage):
        max_voltage = 8.4  # 100%
        min_voltage = 6.0  # 0%

        if voltage >= max_voltage:
            return 100
        elif voltage <= min_voltage:
            return 0
        percentage = (voltage - min_voltage) / (
            max_voltage - min_voltage) * 100
        return round(percentage, 2)

    def read_register(self, register):
        try:
            value = self.bus.read_word_data(
                self.device_address, register)
        except Exception as e:
            print('[Battery Monitor] {}'.format(e))
            return None
        return value

    def calculate_voltage_from_bits(self, bits):
        voltage = 0
        voltage += ((bits & 0x200) >> 9) * 2560
        voltage += ((bits & 0x100) >> 8) * 1280
        voltage += ((bits & 0x080) >> 7) * 640
        voltage += ((bits & 0x040) >> 6) * 320
        voltage += ((bits & 0x020) >> 5) * 160
        voltage += ((bits & 0x010) >> 4) * 80
        voltage += ((bits & 0x008) >> 3) * 40
        voltage += ((bits & 0x004) >> 2) * 20
        voltage += ((bits & 0x002) >> 1) * 10
        voltage += (bits & 0x001) * 5
        return voltage

    def read_charge_status(self):
        """Read charge status

0: No charging
1: Trickle charge
2: Pre-charge
3: CC charge
4: CV charge
5: Charge termination
"""
        reg16_value = self.read_register(0x16)
        if reg16_value is None:
            return None
        charge_status = (reg16_value & 0x1c0) >> 6
        return charge_status

    def read_input_voltage(self):
        reg23_value = self.read_register(0x23)
        if reg23_value is None:
            return None
        input_voltage = 0
        input_voltage += ((reg23_value & 0x200) >> 9) * 10240
        input_voltage += ((reg23_value & 0x100) >> 8) * 5120
        input_voltage += ((reg23_value & 0x080) >> 7) * 2560
        input_voltage += ((reg23_value & 0x040) >> 6) * 1280
        input_voltage += ((reg23_value & 0x020) >> 5) * 640
        input_voltage += ((reg23_value & 0x010) >> 4) * 320
        input_voltage += ((reg23_value & 0x008) >> 3) * 160
        input_voltage += ((reg23_value & 0x004) >> 2) * 80
        input_voltage += ((reg23_value & 0x002) >> 1) * 40
        input_voltage += (reg23_value & 0x001) * 20
        return 0.001 * input_voltage

    def read_system_voltage(self):
        reg26_value = self.read_register(self.REG26H)
        if reg26_value is None:
            return None
        system_voltage = 0
        system_voltage += ((reg26_value & 0x200) >> 9) * 10240
        system_voltage += ((reg26_value & 0x100) >> 8) * 5120
        system_voltage += ((reg26_value & 0x080) >> 7) * 2560
        system_voltage += ((reg26_value & 0x040) >> 6) * 1280
        system_voltage += ((reg26_value & 0x020) >> 5) * 640
        system_voltage += ((reg26_value & 0x010) >> 4) * 320
        system_voltage += ((reg26_value & 0x008) >> 3) * 160
        system_voltage += ((reg26_value & 0x004) >> 2) * 80
        system_voltage += ((reg26_value & 0x002) >> 1) * 40
        system_voltage += (reg26_value & 0x001) * 20
        return 0.001 * system_voltage

    def read_battery_voltage(self):
        reg25_value = self.read_register(self.REG25H)
        if reg25_value is None:
            return None
        battery_voltage = 2 * self.calculate_voltage_from_bits(reg25_value)
        return 0.001 * battery_voltage

    def read_input_current(self, register_address):
        reg24_value = self.read_register(register_address)
        if reg24_value is None:
            return None
        input_current = 0
        input_current += ((reg24_value & 0x200) >> 9) * 3200
        input_current += ((reg24_value & 0x100) >> 8) * 1600
        input_current += ((reg24_value & 0x080) >> 7) * 800
        input_current += ((reg24_value & 0x040) >> 6) * 400
        input_current += ((reg24_value & 0x020) >> 5) * 200
        input_current += ((reg24_value & 0x010) >> 4) * 100
        input_current += ((reg24_value & 0x008) >> 3) * 50
        input_current += ((reg24_value & 0x004) >> 2) * 25
        input_current += ((reg24_value & 0x002) >> 1) * 12.5
        input_current += (reg24_value & 0x001) * 6.25
        return input_current

    def read_junction_temperature(self):
        reg2a_value = self.read_register(0x2A)
        if reg2a_value is None:
            return None
        junction_temp = 0
        junction_temp += ((reg2a_value & 0x200) >> 9) * 512
        junction_temp += ((reg2a_value & 0x100) >> 8) * 256
        junction_temp += ((reg2a_value & 0x080) >> 7) * 128
        junction_temp += ((reg2a_value & 0x040) >> 6) * 64
        junction_temp += ((reg2a_value & 0x020) >> 5) * 32
        junction_temp += ((reg2a_value & 0x010) >> 4) * 16
        junction_temp += ((reg2a_value & 0x008) >> 3) * 8
        junction_temp += ((reg2a_value & 0x004) >> 2) * 4
        junction_temp += ((reg2a_value & 0x002) >> 1) * 2
        junction_temp += (reg2a_value & 0x001) * 1
        return 314 - 0.5703 * junction_temp

    def read_sensor_data(self, get_charge=False):
        is_charging = self.read_charge_status() != 0
        filtered_is_charging = self.get_is_charging()
        if get_charge is True:
            return is_charging
        if filtered_is_charging is False:
            self.set_adc_continuous_mode(set_bit=True)
        battery_voltage = self.read_battery_voltage()
        if filtered_is_charging is False:
            self.set_adc_continuous_mode(set_bit=False)
        if battery_voltage is None:
            return 0
        return self.calculate_lipo_percentage(battery_voltage)

    def run(self):
        try:
            while self.running:
                is_charging = self.read_sensor_data(get_charge=True)
                percentage = self.read_sensor_data()
                if percentage is None or is_charging is None:
                    time.sleep(0.2)
                    continue
                with self.lock:
                    if self.is_outlier(
                            percentage,
                            self.percentage_history,
                            self.percentage_threshold):
                        print("Percentage outlier detected:",
                              f" {percentage:.2f}, ",
                              f"history: {self.percentage_history}")
                    else:
                        self.filtered_percentage = \
                            (self.alpha * percentage
                             + (1 - self.alpha) * self.filtered_percentage)
                    # Always update history
                    self.update_history(percentage, self.percentage_history)

                    self.charging_history.append(is_charging)
                    if len(self.charging_history) > self.history_size:
                        self.charging_history.pop(0)

                if self.debug:
                    print(f"RAW Percentage: {percentage:.2f}")
                    print("Filtered Percentage:",
                          f" {self.filtered_percentage:.2f}")
                time.sleep(0.2)
        finally:
            self.bus.close()

    def get_filtered_percentage(self, charge_status=True):
        with self.lock:
            return self.filtered_percentage

    def get_is_charging(self):
        with self.lock:
            return majority_vote(self.charging_history) == 1

    def stop(self):
        self.running = False
        self.join()


if __name__ == '__main__':
    monitor = MP2760BatteryMonitor(bus_number=3)
    monitor.daemon = True
    monitor.start()

    while True:
        time.sleep(1.0)
        print(monitor.get_is_charging())
