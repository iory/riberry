#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <M5AtomS3.h>

// Dynamixel2Arduino ////////////////////////////////////////////////////////
using namespace ControlTableItem;

#define DXL_SERIAL Serial1
#define SERIAL_CONFIG SERIAL_8N1
const int TIMEOUT = 100;  // ms
const float DXL_PROTOCOL_VERSION = 2.0;

const int DXL_DIR_PIN = 6;  // Enable Pin
const int RX_PIN = 5;
const int TX_PIN = 38;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// 指定可能なBaudrateはemanual(https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/)参照
const long baudrates[] = {
        9600, 57600, 115200, 1000000, 2000000, 3000000, 4000000,
};

void beginDXL(long baudrate) {
    DXL_SERIAL.end();
    DXL_SERIAL.begin(baudrate, SERIAL_CONFIG, RX_PIN, TX_PIN, false, TIMEOUT);
    dxl.begin(baudrate);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
}

// DynamixelのIDとBaudrateを取得
bool scanDXL(uint8_t* ID_found, long* baud_found) {
    for (long baudrate : baudrates) {
        beginDXL(baudrate);
        for (uint8_t id = 0; id < 253; id++) {
            M5.Lcd.clear();
            M5.Lcd.setCursor(0, 0);
            M5.Lcd.println("Scanning..\n");
            M5.Lcd.println("Baudrate ");
            M5.Lcd.println(baudrate);
            M5.Lcd.println("ID ");
            M5.Lcd.println(id);
            if (dxl.ping(id)) {
                *ID_found = id;
                *baud_found = baudrate;
                return true;
            }
        }
        delay(100);
    }
    return false;
}

void waitForButtonPress() {
    // ボタンが押されるのを待つ
    while (!M5.BtnA.wasPressed()) {
        M5.update();
        delay(10);
    }
}

/////////////////////////////////////////////////////////////////////////////

const long TARGET_BAUD = 2000000;
const uint8_t TARGET_ID = 0;

void setup() {
    M5.begin();
    M5.Lcd.setRotation(0);  // 画面向き設定（USB位置基準 0：下/ 1：右/ 2：上/ 3：左）
    M5.Lcd.setTextSize(2);  // 文字サイズ（整数倍率）
    M5.Lcd.print("Dynamixel Set Baudrate");

    // --- Dynamixelをスキャン ---
    uint8_t current_id;
    long current_baud;
    bool scan_succeed = scanDXL(&current_id, &current_baud);

    if (scan_succeed) {
        dxl.torqueOff(current_id);
        M5.Lcd.clear();
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.printf("DXL Found\n\n");
        M5.Lcd.printf("Baudrate\n%ld\n", current_baud);
        M5.Lcd.printf("ID\n%d\n", current_id);
        delay(2000);

        // ボーレートが異なる場合
        if (current_baud != TARGET_BAUD) {
            M5.Lcd.clear();
            M5.Lcd.setCursor(0, 0);
            M5.Lcd.printf("Press to change baud\n\n");
            M5.Lcd.printf("%ld\n->\n%ld", current_baud, TARGET_BAUD);
            waitForButtonPress();
            M5.Lcd.clear();
            M5.Lcd.setCursor(0, 0);
            M5.Lcd.print("Setting Baud...");
            if (dxl.setBaudrate(current_id, TARGET_BAUD)) {
                M5.Lcd.clear();
                M5.Lcd.setCursor(0, 0);
                M5.Lcd.print("SUCCESS\n\nReboot device to apply change.");
            } else {
                M5.Lcd.clear();
                M5.Lcd.setCursor(0, 0);
                M5.Lcd.print("FAILED\n\nRestart device and try again.");
            }
        }
        // ボーレートが同じで、IDが異なる場合
        else if (current_id != TARGET_ID) {
            M5.Lcd.clear();
            M5.Lcd.setCursor(0, 0);
            M5.Lcd.printf("Press to change ID\n\n");
            M5.Lcd.printf("%d\n->\n%d", current_id, TARGET_ID);
            waitForButtonPress();
            if (dxl.setID(current_id, TARGET_ID)) {
                M5.Lcd.clear();
                M5.Lcd.setCursor(0, 0);
                M5.Lcd.print("SUCCESS\n\nReboot device to apply change.");
            } else {
                M5.Lcd.clear();
                M5.Lcd.setCursor(0, 0);
                M5.Lcd.print("FAILED\n\nRestart device and try again.");
            }
        }
        // ボーレートもIDも同じ場合
        else {
            M5.Lcd.clear();
            M5.Lcd.setCursor(0, 0);
            M5.Lcd.print("Setting OK\n\n");
            M5.Lcd.printf("Baudrate\n%ld\n", current_baud);
            M5.Lcd.printf("ID\n%d\n", current_id);
        }
    } else {
        M5.Lcd.clear();
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.print("DXL not found.");
    }
}

void loop() { delay(100); }
