// Mainly copied from dynamixe_current.cpp
#include <Dynamixel2Arduino.h>
#define DXL_SERIAL Serial1
#define SERIAL_CONFIG SERIAL_8N1
const long BAUDRATE = 2000000;  // 2Mが限界だった
const int TIMEOUT = 100;        // ms
const uint8_t DXL_ID = 0;
const float DXL_PROTOCOL_VERSION = 2.0;

const int EN_PIN = 6;  // Enable Pin
const int RX_PIN = 5;
const int TX_PIN = 38;

Dynamixel2Arduino dxl(DXL_SERIAL, EN_PIN);

void setupDXL() {
    DXL_SERIAL.begin(BAUDRATE, SERIAL_CONFIG, RX_PIN, TX_PIN, false, TIMEOUT);
    dxl.begin(BAUDRATE);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

    dxl.torqueOff(DXL_ID);
    dxl.setOperatingMode(DXL_ID, OP_CURRENT);
    dxl.torqueOn(DXL_ID);

    dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, DXL_ID, 0);
}

// 電圧を読み取る関数
float readVoltage() {
    int32_t voltage_raw = dxl.readControlTableItem(ControlTableItem::PRESENT_INPUT_VOLTAGE, DXL_ID);
    return (float)voltage_raw / 10.0;  // Unit [V]
}

// 温度を読み取る関数
float readTemperature() {
    int32_t temperature = dxl.readControlTableItem(ControlTableItem::PRESENT_TEMPERATURE, DXL_ID);
    return (float)temperature;  // Unit [C]
}

// Mainly copied from esp_now_pairing
#define LGFX_M5ATOMS3
#define LGFX_USE_V1
#include <SerialTransfer.h>

#include <LGFX_AUTODETECT.hpp>
#include <LovyanGFX.hpp>

#include "pairing.h"

#ifdef ENV_MAIN
String main_or_secondary = "Main";
#else
String main_or_secondary = "Secondary";
#endif

static LGFX lcd;
const int channel = 1;  // To avoid interference, channels should be at least 3 apart.
Pairing pairing(channel);
String previousMessage = "";
SerialTransfer transfer;
uint8_t buffer[256];

// 表示更新用の変数
unsigned long lastDisplayTime = 0;
const unsigned long displayInterval = 500;  // 表示更新間隔 [ms]
String currentStatusMessage = "";           // 現在の状態メッセージを保持する変数

template <typename T>
void write(const T &data, const size_t len) {
    transfer.txObj(data, 0, len);
    transfer.sendData(len);
}

String getBaseDisplayMessage() {
    return main_or_secondary + " ch." + String(channel) + "\nMy MAC:\n" + pairing.getMyMACAddress();
}

void printToLCD(const String &message) {
    if (message == previousMessage) {
        return;
    }
    previousMessage = message;
    lcd.fillScreen(lcd.color565(0, 0, 0));
    lcd.setCursor(0, 0);
    lcd.println(message);
}

void initLCD() {
    lcd.init();
    lcd.setRotation(0);
    lcd.clear();
    lcd.setTextSize(1.2);
}

void setup() {
    initLCD();
    currentStatusMessage = "Initializing...";
    printToLCD(currentStatusMessage);

    int xCoreID = 1;
    pairing.createTask(xCoreID);
    String baseMessage = getBaseDisplayMessage();
    currentStatusMessage = baseMessage;
    printToLCD(currentStatusMessage);

    USBSerial.begin(115200);
    transfer.begin(USBSerial);
    delay(1500);

    if (main_or_secondary == "Main") {
        currentStatusMessage =
                baseMessage + "\n\nWait for command from espnow_dynamixel_controller.py";
        printToLCD(currentStatusMessage);
    } else if (main_or_secondary == "Secondary") {
        setupDXL();
        currentStatusMessage = baseMessage + "\n\nWait for command from Main";
        printToLCD(currentStatusMessage);
    }
}

void control_dynamixel(PairingData data) {
    String baseMessage = getBaseDisplayMessage();
    if (data.IPv4[0] == 0) {
        dxl.setGoalCurrent(DXL_ID, 500, UNIT_MILLI_AMPERE);
        currentStatusMessage = baseMessage + "\n\nForward rotation";
    } else if (data.IPv4[0] == 127) {
        dxl.setGoalCurrent(DXL_ID, 0, UNIT_MILLI_AMPERE);
        currentStatusMessage = baseMessage + "\n\nStop rotation";
    } else if (data.IPv4[0] == 255) {
        dxl.setGoalCurrent(DXL_ID, -800, UNIT_MILLI_AMPERE);
        currentStatusMessage = baseMessage + "\n\nReverse rotation";
    } else {
        currentStatusMessage = baseMessage + "\n\nUnknown command: " + String(data.IPv4[0]);
    }
}

PairingData dataToSend = {{255, 255, 255, 255}};

void loop() {
    size_t available = transfer.available();
    if (available > 0) {
        transfer.rxObj(buffer, 0, available);
        switch (buffer[0]) {
            case 0x11:
                write(main_or_secondary, main_or_secondary.length());
                break;
            case 0x12: {
                std::map<String, PairingData> pairedDataMap = pairing.getPairedData();
                if (!pairedDataMap.empty()) {
                    auto it = pairedDataMap.begin();
                    write(it->second.IPv4, 4);
                }
                break;
            }
            case 0x13:
                for (int i = 0; i < 4; i++) {
                    dataToSend.IPv4[i] = buffer[i + 1];
                }
                pairing.setDataToSend(dataToSend);
                break;
            default:
                break;
        }
        currentStatusMessage = getBaseDisplayMessage() + "\n\nReceive command from computer";
    }

    std::map<String, PairingData> pairedDataMap = pairing.getPairedData();
    for (const auto &pair : pairedDataMap) {
        if (main_or_secondary == "Secondary") {
            control_dynamixel(pair.second);
        }
    }

    unsigned long currentTime = millis();
    if (currentTime - lastDisplayTime >= displayInterval) {
        // Mainデバイスの場合、現在のステータスメッセージのみ表示
        if (main_or_secondary == "Main") {
            printToLCD(currentStatusMessage);
        }
        // Secondaryデバイスの場合、現在のステータスメッセージとモーター情報を結合して表示
        else {
            String motorInfo;
            motorInfo = String("\n\n") + String(readVoltage(), 2) + " [V]\n" +
                        String((int)dxl.getPresentCurrent(DXL_ID, UNIT_MILLI_AMPERE)) + " [mA]\n" +
                        String((int)readTemperature()) + " [C]";
            printToLCD(currentStatusMessage + motorInfo);
        }
        lastDisplayTime = currentTime;
    }

    delay(10);
}
