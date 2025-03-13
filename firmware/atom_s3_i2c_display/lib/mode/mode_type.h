#ifndef MODE_TYPE_H
#define MODE_TYPE_H

namespace ModeType {
enum Name : uint8_t {
    NONE = 0x00,
    DISPLAY_INFORMATION = 0x01,
    DISPLAY_QRCODE = 0x02,
    DISPLAY_IMAGE = 0x03,
    DISPLAY_BATTERY_GRAPH = 0x04,
    DISPLAY_ODOM = 0x05,
    SERVO_CONTROL = 0x06,
    PRESSURE_CONTROL = 0x07,
    TEACHING = 0x08,
    SPEECH_TO_TEXT = 0x09,
    SYSTEM_DEBUG = 0x10,
    PAIRING = 0x11,
    WIFI_SETTINGS = 0x12,

    FIRMWARE_UPDATE = 0xFF,
};

inline String toString(Name name) {
    switch (name) {
        case NONE:
            return "None";
        case DISPLAY_INFORMATION:
            return "DisplayInformationMode";
        case DISPLAY_QRCODE:
            return "DisplayQRcodeMode";
        case DISPLAY_IMAGE:
            return "DisplayImageMode";
        case DISPLAY_BATTERY_GRAPH:
            return "DisplayBatteryGraphMode";
        case DISPLAY_ODOM:
            return "DisplayOdomMode";
        case SERVO_CONTROL:
            return "ServoControlMode";
        case PRESSURE_CONTROL:
            return "PressureControlMode";
        case TEACHING:
            return "TeachingMode";
        case SPEECH_TO_TEXT:
            return "SpeechToTextMode";
        case SYSTEM_DEBUG:
            return "SystemDebugMode";
        case PAIRING:
            return "PairingMode";
        case WIFI_SETTINGS:
            return "WiFiSettingsMode";
        case FIRMWARE_UPDATE:
            return "FirmwareUpdateMode";
        default:
            return "Unknown";
    }
}
}  // namespace ModeType

#endif  // MODE_TYPE_H
