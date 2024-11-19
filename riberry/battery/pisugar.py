import threading
import time

import smbus2

from riberry.battery.common import majority_vote

IP5209_CURVE = [
    (4160, 100),
    (4050, 95),
    (4000, 80),
    (3920, 65),
    (3860, 40),
    (3790, 25),
    (3660, 10),
    (3520, 6),
    (3490, 3),
    (3100, 0),
]


class PisugarBatteryReader(threading.Thread):
    def __init__(
        self,
        bus_number=None,
        device_address=0x57,
        alpha=0.9,
        value_threshold=1000,
        percentage_threshold=20,
        history_size=10,
        debug=False,
    ):
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

        self.vol_history = []

        self.bus = smbus2.SMBus(self.bus_number)
        self.lock = threading.Lock()
        self.running = True

    def __del__(self):
        print("[PisugarBatteryReader] Object is being deleted, cleaning up...")
        self.stop()
        self.bus.close()

    def read_sensor_data(self, get_charge=False):
        try:
            if get_charge is True:
                value = self.bus.read_byte_data(self.device_address, 0x02) >> 7
            else:
                value = self.bus.read_byte_data(self.device_address, 0x2A)
            return value
        except OSError:
            # for pisugar2
            try:
                vol_low = self.bus.read_byte_data(0x75, 0xA2)
                vol_high = self.bus.read_byte_data(0x75, 0xA3)
                if not (0 <= vol_low <= 255 and 0 <= vol_high <= 255):
                    print("[Pisugar Battery Reader] Invalid voltage data")
                    return
                if (vol_high & 0x20) == 0x20:
                    vol = 2600 - (~vol_low + (~(vol_high & 0x1F)) * 256 + 1) * 27 // 100
                else:
                    vol = 2600 + (vol_low + vol_high * 256) * 27 // 100

                self._update_voltage_history(vol)
                is_increasing = self._is_voltage_increasing_trend()
                if get_charge is True:
                    return is_increasing

                cap = 0
                for i in range(len(IP5209_CURVE)):
                    if vol >= IP5209_CURVE[i][0]:
                        cap = IP5209_CURVE[i][1]
                        if i == 0:
                            break
                        if i > 0:
                            vol_diff_v = vol - IP5209_CURVE[i][0]
                            k_percent = IP5209_CURVE[i - 1][1] - IP5209_CURVE[i][1]
                            k_voltage = IP5209_CURVE[i - 1][0] - IP5209_CURVE[i][0]
                            k = k_percent / k_voltage
                            cap += int(k * vol_diff_v)
                            break
                return cap
            except Exception as e:
                print(f"[Pisugar Battery Reader] {e}")
                return None
        except Exception as e:
            print(f"[Pisugar Battery Reader] {e}")
            return None

    def _update_voltage_history(self, vol):
        self.vol_history.append(vol)
        if len(self.vol_history) > self.history_size:
            self.vol_history.pop(0)

    def _is_voltage_increasing_trend(self):
        if len(self.vol_history) < 2:
            return False
        total_slope = 0
        for i in range(1, len(self.vol_history)):
            total_slope += self.vol_history[i] - self.vol_history[i - 1]
        average_slope = total_slope / (len(self.vol_history) - 1)
        return average_slope > 0

    def is_outlier(self, current, history, threshold):
        if not history:
            return False
        ratio = sum(abs(current - h) > threshold for h in history) / len(history)
        return ratio > 0.4

    def update_history(self, value, history):
        history.append(value)
        if len(history) > self.history_size:
            history.pop(0)

    def run(self):
        try:
            while self.running:
                percentage = self.read_sensor_data()
                is_charging = self.read_sensor_data(get_charge=True)
                if percentage is None or is_charging is None:
                    time.sleep(0.2)
                    continue

                with self.lock:
                    if self.is_outlier(
                        percentage, self.percentage_history, self.percentage_threshold
                    ):
                        print(
                            "Percentage outlier detected:",
                            f" {percentage:.2f}, ",
                            f"history: {self.percentage_history}",
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
                    print(f"RAW Percentage: {percentage:.2f}")
                    print("Filtered Percentage:", f" {self.filtered_percentage:.2f}")
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

    @staticmethod
    def exists(self, bus_number=3):
        try:
            with smbus2.SMBus(bus_number) as bus:
                bus.read_byte(0x57)
            print("[PisugarBatteryReader] pisugar3 found.")
            return True
        except OSError:
            pass
        try:
            with smbus2.SMBus(bus_number) as bus:
                bus.read_byte(0x75)
            print("[PisugarBatteryReader] pisugar2 found.")
            return True
        except OSError:
            return False
        return False
