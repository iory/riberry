from enum import Enum


class ModeType(Enum):
    NONE = 0x00
    DISPLAY_INFORMATION = 0x01
    DISPLAY_QRCODE = 0x02
    DISPLAY_IMAGE = 0x03
    DISPLAY_BATTERY_GRAPH = 0x04
    DISPLAY_ODOM = 0x05
    SERVO_CONTROL = 0x06
    PRESSURE_CONTROL = 0x07
    TEACHING = 0x08
    SPEECH_TO_TEXT = 0x09
    SYSTEM_DEBUG = 0x10
    PAIRING = 0x11


def mode_type_to_string(mode_type):
    if mode_type == ModeType.NONE:
        return "None"
    elif mode_type == ModeType.DISPLAY_INFORMATION:
        return "DisplayInformationMode"
    elif mode_type == ModeType.DISPLAY_QRCODE:
        return "DisplayQRcodeMode"
    elif mode_type == ModeType.DISPLAY_IMAGE:
        return "DisplayImageMode"
    elif mode_type == ModeType.DISPLAY_BATTERY_GRAPH:
        return "DisplayBatteryGraphMode"
    elif mode_type == ModeType.DISPLAY_ODOM:
        return "DisplayOdomMode"
    elif mode_type == ModeType.SERVO_CONTROL:
        return "ServoControlMode"
    elif mode_type == ModeType.PRESSURE_CONTROL:
        return "PressureControlMode"
    elif mode_type == ModeType.TEACHING:
        return "TeachingMode"
    elif mode_type == ModeType.SPEECH_TO_TEXT:
        return "SpeechToTextMode"
    elif mode_type == ModeType.SYSTEM_DEBUG:
        return "SystemDebugMode"
    elif mode_type == ModeType.PAIRING:
        return "PairingMode"
    else:
        return "Unknown"
