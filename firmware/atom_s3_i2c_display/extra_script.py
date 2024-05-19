import os
from SCons.Script import Import

Import("env")

USE_GROVE = os.getenv("USE_GROVE")
LCD_ROTATION = os.getenv("LCD_ROTATION")

if USE_GROVE == "1":
    env.Append(CPPDEFINES=["USE_GROVE"])

if LCD_ROTATION:
    if LCD_ROTATION not in ["0", "1", "2", "3"]:
        print("\033[91mWarning: Invalid LCD_ROTATION value: {}. It should be 0, 1, 2, or 3.\033[0m".format(LCD_ROTATION))
        Exit(1)
    env.Append(CPPDEFINES=[f"LCD_ROTATION={LCD_ROTATION}"])
else:
    env.Append(CPPDEFINES=["LCD_ROTATION=1"])
