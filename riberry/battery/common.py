from collections import Counter

from riberry.i2c_base import I2CBase


def majority_vote(history):
    if not history:
        return 0
    count = Counter(history)
    return count.most_common(1)[0][0]


def decide_battery_i2c_bus_number():
    device_type = I2CBase.identify_device()
    if device_type == "Raspberry Pi":
        bus_number = 1
    elif device_type == "Radxa Zero":
        bus_number = 3
    else:
        bus_number = None
    return bus_number
