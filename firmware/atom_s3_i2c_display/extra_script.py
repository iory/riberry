# flake8: noqa
import os

from SCons.Script import Import


def validate_i2c_address(addr):
    if not addr.startswith("0x"):
        print("\033[91mWarning: I2C address does not start with '0x'.\033[0m")
        return False
    try:
        address_int = int(addr, 16)
    except ValueError:
        print("\033[91mWarning: I2C address is not a valid hexadecimal number.\033[0m")
        return False

    if not (0x03 <= address_int <= 0x77):
        print("\033[91mWarning: I2C address is out of valid range (0x03 to 0x77).\033[0m")
        return False

    return True


Import("env")

USE_GROVE = os.getenv("USE_GROVE")
LCD_ROTATION = os.getenv("LCD_ROTATION")
I2C_ADDR = os.getenv("I2C_ADDR")
USE_USB_SERIAL = os.getenv("USE_USB_SERIAL")

if USE_GROVE == "1":
    env.Append(CPPDEFINES=["USE_GROVE"])

if I2C_ADDR:
    if validate_i2c_address(I2C_ADDR):
        env.Append(CPPDEFINES=[f"I2C_ADDR={int(I2C_ADDR, 16)}"])
    else:
        print("\033[91mUsing default I2C address 0x42.\033[0m")
        env.Append(CPPDEFINES=["I2C_ADDR=0x42"])

if LCD_ROTATION:
    if LCD_ROTATION not in ["0", "1", "2", "3"]:
        print(
            f"\033[91mWarning: Invalid LCD_ROTATION value: {LCD_ROTATION}. It should be 0, 1, 2, or 3.\033[0m"
        )
        Exit(1)
    env.Append(CPPDEFINES=[f"LCD_ROTATION={LCD_ROTATION}"])
else:
    env.Append(CPPDEFINES=["LCD_ROTATION=1"])

if USE_USB_SERIAL == "1":
    env.Append(CPPDEFINES=["USE_USB_SERIAL"])
