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
    WIFI_SETTINGS = 0x12

    FIRMWARE_UPDATE_MODE = 0xFF


MODE_TYPE_MAPPING = {
    ModeType.NONE: "None",
    ModeType.DISPLAY_INFORMATION: "DisplayInformationMode",
    ModeType.DISPLAY_QRCODE: "DisplayQRcodeMode",
    ModeType.DISPLAY_IMAGE: "DisplayImageMode",
    ModeType.DISPLAY_BATTERY_GRAPH: "DisplayBatteryGraphMode",
    ModeType.DISPLAY_ODOM: "DisplayOdomMode",
    ModeType.SERVO_CONTROL: "ServoControlMode",
    ModeType.PRESSURE_CONTROL: "PressureControlMode",
    ModeType.TEACHING: "TeachingMode",
    ModeType.SPEECH_TO_TEXT: "SpeechToTextMode",
    ModeType.SYSTEM_DEBUG: "SystemDebugMode",
    ModeType.PAIRING: "PairingMode",
    ModeType.WIFI_SETTINGS: "WiFiSettingsMode",

    ModeType.FIRMWARE_UPDATE_MODE: "FirmwareUpdateMode",
}



def mode_type_to_string(mode_type):
    try:
        return MODE_TYPE_MAPPING.get(mode_type)
    except ValueError:
        return "Unknown"


def string_to_mode_type(mode_str):
    reverse_mapping = {v: k for k, v in MODE_TYPE_MAPPING.items()}
    return reverse_mapping.get(mode_str, ModeType.NONE)
