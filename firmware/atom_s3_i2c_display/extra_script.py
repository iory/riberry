import os
from SCons.Script import Import

Import("env")

USE_GROVE = os.getenv("USE_GROVE")

if USE_GROVE == "1":
    env.Append(CPPDEFINES=["USE_GROVE"])
