from enum import Enum
import logging
import os
import threading
import time

import smbus2

from riberry.battery.common import majority_vote

# Set up logging
LOG_LEVEL = os.getenv('LOG_LEVEL', 'INFO').upper()
logging.basicConfig(level=LOG_LEVEL)
logging.getLogger("filelock").setLevel(logging.WARNING)
logger = logging.getLogger(__name__)

class ChargeState(Enum):
    NO_CHARGE = 0
    TRICKLE_CHARGE = 1
    PRE_CHARGE = 2
    CC_CHARGE = 3
    CV_CHARGE = 4
    CHARGE_TERMINATION = 5

    def __str__(self):
        return {
            ChargeState.NO_CHARGE: "No charging",
            ChargeState.TRICKLE_CHARGE: "Trickle charge",
            ChargeState.PRE_CHARGE: "Pre-charge",
            ChargeState.CC_CHARGE: "CC charge",
            ChargeState.CV_CHARGE: "CV charge",
            ChargeState.CHARGE_TERMINATION: "Charge termination",
        }[self]


def print_status_and_fault_register(value):
    if value is None:
        return "Could not read register."
    # Ensure the input is 16 bits
    if value > 0xFFFF or value < 0:
        raise ValueError("Input must be a 16-bit value (0x0000 - 0xFFFF).")

    # Bit descriptions based on the table from the image
    descriptions = {
        15: "VIN_SRC_OV: Output OVP in source mode",
        14: "VIN_SRC_UV: Output UVP in source mode",
        13: "VIN_CHG_OV: Input OVP in charge mode",
        12: "VADP_OV: ADP OVP",
        11: "VSYS_OV: System OV in charging mode",
        10: "VSYS_UV: System UV in charging mode",
        # 9: "Reserved",  # No need to print Reserved bits
        # 8: "Reserved",
        7: "VBATT_LOW: Low battery voltage, discharge stop",
        6: "WTD_EXP: Watchdog timer expiration",
        5: "CHG_TMR_EXP: Charge safety timer expiration",
        4: "THERM_SHDN: Thermal shutdown",
        # 3: "Reserved",
        2: "NTC_FAULT[2]: NTC fault 2",
        1: "NTC_FAULT[1]: NTC fault 1",
        0: "NTC_FAULT[0]: NTC fault 0",
    }

    # Iterate through the bits and print descriptions for the set bits
    status_and_fault_list = []
    for bit in range(15, -1, -1):
        if bit not in descriptions:
            continue
        if (value >> bit) & 1:
            msg = f"Bit {bit}: {descriptions.get(bit, 'Unknown')}"
            logger.info(msg)
            status_and_fault_list.append(msg)
    if len(status_and_fault_list) == 0:
        return "All green."
    all_msg = "\n".join(status_and_fault_list)
    return all_msg


class MP2760BatteryMonitor(threading.Thread):
    REG27H = 0x27  # Battery Charge Current
    REG26H = 0x26  # System Voltage
    REG24H = 0x24  # Input Current
    REG25H = 0x25  # Battery Voltage

    def __init__(
        self,
        bus_number=3,
        device_address=0x5C,
        alpha=0.9,
        percentage_threshold=20,
        history_size=10,
        debug=True,
    ):
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
        while self.disable_ntc_protection() is None:
            logger.warning("[MP2760BatteryMonitor] Try to disable NTC protection.")
            time.sleep(1.0)
        while self.set_adc_continuous_mode(set_bit=True) is None:
            logger.warning("[MP2760BatteryMonitor] Try to enable adc continuous mode.")
            time.sleep(1.0)
        while self.set_safety_timer(set_bit=True) is None:
            logger.warning("[MP2760BatteryMonitor] Try to enable safety timer.")
            time.sleep(1.0)
        self.limit_charge_current(400)
        logger.info(
            "[MP2760BatteryMonitor] Charge current limit: " +
            f"{self.read_charge_current_limit()}[mA]"
        )
        self.limit_input_current(500)
        logger.info(
            "[MP2760BatteryMonitor] Input current limit: " +
            f"{self.read_input_current_limit()}[mA]"
        )
        self.lock = threading.Lock()
        self.running = True

    def __del__(self):
        logger.info("[MP2760BatteryMonitor] Object is being deleted, cleaning up...")
        self.stop()
        self.set_adc_continuous_mode(False)
        self.bus.close()

    @staticmethod
    def exists(self, bus_number=3, device_address=0x5C):
        try:
            with smbus2.SMBus(bus_number) as bus:
                bus.read_byte(device_address)
            logger.info("[MP2760BatteryMonitor] found.")
            return True
        except OSError as e:
            logger.error(f"[MP2760BatteryMonitor] {e}. Device not found")
            return False

    def is_outlier(self, current, history, threshold):
        if not history:
            return False
        ratio = sum(abs(current - h) > threshold for h in history) / len(history)
        return ratio > 0.4

    def update_history(self, value, history):
        history.append(value)
        if len(history) > self.history_size:
            history.pop(0)

    def set_adc_continuous_mode(self, set_bit=True):
        try:
            word = self.bus.read_word_data(self.device_address, 0x0E)
        except Exception as e:
            logger.error(f"[Battery Monitor] Error reading from I2C: {e}")
            return
        current_bit = (word >> 7) & 1
        if current_bit == set_bit:
            logger.info(
                "[Battery Monitor] 7bit is already set to the desired value. "
                + "No action needed."
            )
            return True
        if set_bit:
            logger.info("[Battery Monitor] Set ADC_CONV to Continuous")
            set_adc_word = word | (1 << 7)
        else:
            logger.info("[Battery Monitor] Set ADC_CONV to One-shot conversion")
            set_adc_word = word & ~(1 << 7)
        try:
            self.bus.write_word_data(self.device_address, 0x0E, set_adc_word)
        except Exception as e:
            logger.error(f"[Battery Monitor] Error writing I2C: {e}")
            return
        return True

    def disable_ntc_protection(self):
        try:
            word = self.bus.read_word_data(self.device_address, 0x0D)
        except Exception as e:
            logger.error(f"[Battery Monitor] Error reading from I2C: {e}")
            return
        current_bit = (word >> 9) & 1
        if current_bit == 0:
            logger.info(
                "[Battery Monitor] 9bit is already set to the desired value. "
                + "No action needed."
            )
            return True
        logger.info("[Battery Monitor] Disable NTC protection")
        set_word = word & ~(1 << 9)
        try:
            self.bus.write_word_data(self.device_address, 0x0D, set_word)
        except Exception as e:
            logger.error(f"[Battery Monitor] Error writing I2C: {e}")
            return
        return True

    def set_safety_timer(self, set_bit=True):
        try:
            word = self.bus.read_word_data(self.device_address, 0x12)
        except Exception as e:
            logger.error(f"[Battery Monitor] Error reading from I2C: {e}")
            return
        if set_bit is True:
            logger.info("[Battery Monitor] Safety timer is enabled.")
            set_word = word | (1 << 13)
        else:
            logger.info("[Battery Monitor] Safety timer is disabled.")
            set_word = word & ~(1 << 13)
        try:
            self.bus.write_word_data(self.device_address, 0x12, set_word)
        except Exception as e:
            logger.error(f"[Battery Monitor] Error writing I2C: {e}")
            return
        return True

    def limit_charge_current(self, set_current):  # unit: [mA]
        set_word = 0x0000
        digits = list(range(13, 5, -1))
        currents = [50 * 2**i for i in range(7, -1, -1)]
        if (
            type(set_current) is not int
            or set_current % 50 != 0
            or set_current < 50
            or set_current > 6000
        ):
            logger.error("[Battery Monitor] Charge current limit is not proper.")
            return
        for digit, current in zip(digits, currents):
            bit = set_current // current
            set_word += bit << digit
            set_current -= bit * 50 * 2 ** (digit - 6)
        try:
            self.bus.write_word_data(self.device_address, 0x14, set_word)
        except Exception as e:
            logger.error(f"[Battery Monitor] Error writing I2C: {e}")
            return
        return True

    def limit_input_current(self, set_current):  # unit: [mA]
        set_word = 0x0000
        digits = list(range(6, -1, -1))
        currents = [50 * 2**i for i in range(6, -1, -1)]
        if (
            type(set_current) is not int
            or set_current % 50 != 0
            or set_current < 50
            or set_current > 5000
        ):
            logger.error("[Battery Monitor] Input current limit is not proper.")
            return
        for digit, current in zip(digits, currents):
            bit = set_current // current
            set_word += bit << digit
            set_current -= bit * 50 * 2 ** (digit - 0)
        try:
            self.bus.write_word_data(self.device_address, 0x08, set_word)
        except Exception as e:
            logger.error(f"[Battery Monitor] Error writing I2C: {e}")
            return
        return True

    def write_default_value_to_0x10_register(self):
        try:
            self.bus.write_word_data(self.device_address, 0x10, 0x0A74)
            logger.info("[Battery Monitor] Successfully wrote 0x0A74 to register 0x10.")
            return True
        except Exception as e:
            logger.error(f"[Battery Monitor] Error writing to register 0x10: {e}")
            return False

    def read_charge_current_limit(self):  # unit: [mA]
        try:
            bits = self.bus.read_word_data(self.device_address, 0x14)
        except Exception as e:
            logger.error(f"[Battery Monitor] Error reading from I2C: {e}")
            return
        current = 0
        current += ((bits & 0x2000) >> 13) * 6400
        current += ((bits & 0x1000) >> 12) * 3200
        current += ((bits & 0x0800) >> 11) * 1600
        current += ((bits & 0x0400) >> 10) * 800
        current += ((bits & 0x0200) >> 9) * 400
        current += ((bits & 0x0100) >> 8) * 200
        current += ((bits & 0x0080) >> 7) * 100
        current += ((bits & 0x0040) >> 6) * 50
        return current

    def read_input_current_limit(self):  # unit: [mA]
        try:
            bits = self.bus.read_word_data(self.device_address, 0x08)
        except Exception as e:
            logger.error(f"[Battery Monitor] Error reading from I2C: {e}")
            return
        current = 0
        current += ((bits & 0x0040) >> 6) * 3200
        current += ((bits & 0x0020) >> 5) * 1600
        current += ((bits & 0x0010) >> 4) * 800
        current += ((bits & 0x0008) >> 3) * 400
        current += ((bits & 0x0004) >> 2) * 200
        current += ((bits & 0x0002) >> 1) * 100
        current += ((bits & 0x0001) >> 0) * 50
        return current

    def calculate_lipo_percentage(self, voltage):
        max_voltage = 8.4  # 100%
        min_voltage = 6.4  # 0%

        if voltage >= max_voltage:
            return 100
        elif voltage <= min_voltage:
            return 0
        percentage = (voltage - min_voltage) / (max_voltage - min_voltage) * 100
        return round(percentage, 2)

    def read_register(self, register):
        try:
            value = self.bus.read_word_data(self.device_address, register)
        except Exception as e:
            logger.error(f"[Battery Monitor] {e}")
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
        charge_status_value = (reg16_value & 0x1C0) >> 6
        try:
            charge_status = ChargeState(charge_status_value)
        except ValueError:
            return ChargeState(0)
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

    def read_battery_charge_current(self):
        reg27_value = self.read_register(0x27)
        if reg27_value is None:
            return 0
        input_current = 0
        input_current += ((reg27_value & 0x200) >> 9) * 6400
        input_current += ((reg27_value & 0x100) >> 8) * 3200
        input_current += ((reg27_value & 0x080) >> 7) * 1600
        input_current += ((reg27_value & 0x040) >> 6) * 800
        input_current += ((reg27_value & 0x020) >> 5) * 400
        input_current += ((reg27_value & 0x010) >> 4) * 200
        input_current += ((reg27_value & 0x008) >> 3) * 100
        input_current += ((reg27_value & 0x004) >> 2) * 50
        input_current += ((reg27_value & 0x002) >> 1) * 25
        input_current += (reg27_value & 0x001) * 12.5
        return input_current

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
        charge_status = self.read_charge_status()
        is_charging = charge_status is not None and charge_status.value != 0
        if get_charge is True:
            return is_charging
        input_voltage = self.read_input_voltage()
        system_voltage = self.read_system_voltage()
        battery_voltage = self.read_battery_voltage()
        battery_charge_current = self.read_battery_charge_current()
        temp = self.read_junction_temperature()
        if battery_voltage is None:
            return (
                input_voltage,
                system_voltage,
                0,
                battery_voltage,
                temp,
                charge_status,
                battery_charge_current,
            )
        return (
            input_voltage,
            system_voltage,
            self.calculate_lipo_percentage(battery_voltage),
            battery_voltage,
            temp,
            charge_status,
            battery_charge_current,
        )

    def run(self):
        try:
            loop_count = 0
            while self.running:
                loop_count += 1
                if loop_count % 100 == 0:
                    self.write_default_value_to_0x10_register()
                    loop_count = 0
                self.status_and_fault = self.read_register(0x17)
                self.status_and_fault_string = print_status_and_fault_register(
                    self.status_and_fault
                )
                is_charging = self.read_sensor_data(get_charge=True)
                (
                    self.input_voltage,
                    self.system_voltage,
                    percentage,
                    self.battery_voltage,
                    self.junction_temperature,
                    self.charge_status,
                    self.battery_charge_current,
                ) = self.read_sensor_data()
                if self.input_voltage:
                    logger.info(f"Input Voltage: {self.input_voltage:.2f} V")
                if self.system_voltage:
                    logger.info(f"System Voltage: {self.system_voltage:.2f} V")
                if self.battery_voltage:
                    logger.info(f"Battery Voltage: {self.battery_voltage:.2f} V")
                logger.info(f"Battery charge Current: {self.battery_charge_current:.2f} mA")
                if self.junction_temperature:
                    logger.info(f"Junction Temperature: {self.junction_temperature:.2f}")
                if percentage is None or is_charging is None:
                    time.sleep(0.2)
                    continue
                with self.lock:
                    if self.is_outlier(
                        percentage, self.percentage_history, self.percentage_threshold
                    ):
                        logger.warning(
                            f"Percentage outlier detected: {percentage:.2f}, "
                            + f"history: {self.percentage_history}"
                        )
                    else:
                        self.filtered_percentage = (
                            self.alpha * percentage
                            + (1 - self.alpha) * self.filtered_percentage
                        )
                    # Always update history
                    self.update_history(percentage, self.percentage_history)

                    self.charging_history.append(is_charging)
                    if len(self.charging_history) > self.history_size:
                        self.charging_history.pop(0)

                if self.debug:
                    logger.debug(f"RAW Percentage: {percentage:.2f}")
                    logger.debug(f"Filtered Percentage: {self.filtered_percentage:.2f}")
                    logger.debug(f"Charge Status: {self.charge_status}")
                time.sleep(0.2)
        finally:
            self.bus.close()

    def get_filtered_percentage(self, charge_status=True):
        with self.lock:
            return self.filtered_percentage

    def get_is_charging(self):
        with self.lock:
            return majority_vote(self.charging_history) == 1

    def get_charge_status(self):
        return self.charge_status

    def stop(self):
        self.running = False
        self.join()


if __name__ == "__main__":
    monitor = MP2760BatteryMonitor(bus_number=3)
    monitor.daemon = True
    monitor.start()

    while True:
        time.sleep(1.0)
        print(monitor.get_is_charging())
