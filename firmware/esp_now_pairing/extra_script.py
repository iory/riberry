# flake8: noqa
import os

from SCons.Script import Import


Import("env")

PAIRING_TYPE = os.getenv("PAIRING_TYPE")
if PAIRING_TYPE is not None:
    env.Append(CPPDEFINES=[PAIRING_TYPE])
