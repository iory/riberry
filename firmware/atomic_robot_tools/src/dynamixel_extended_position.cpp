#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <M5AtomS3.h>

// Dynamixel2Arduino ////////////////////////////////////////////////////////
#define DXL_SERIAL Serial1
#define SERIAL_CONFIG SERIAL_8N1

using namespace ControlTableItem;

const long BAUDRATE = 57600;
const int TIMEOUT = 100;  // ms
const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 2.0;

const int EN_PIN = 6;  // Enable Pin
const int RX_PIN = 5;
const int TX_PIN = 38;

Dynamixel2Arduino dxl(DXL_SERIAL, EN_PIN);

void setupDXL() {
    DXL_SERIAL.begin(BAUDRATE, SERIAL_CONFIG, RX_PIN, TX_PIN, false, TIMEOUT);
    dxl.begin(BAUDRATE);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

    // 定格以上の電圧をかけるとShutdownがかかってしまうので、ここで無理やりビットフラグを0にする
    dxl.writeControlTableItem(ControlTableItem::SHUTDOWN, DXL_ID, 0);

    dxl.torqueOff(DXL_ID);
    dxl.setOperatingMode(DXL_ID, OP_EXTENDED_POSITION);
    dxl.torqueOn(DXL_ID);

    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 0);
}
/////////////////////////////////////////////////////////////////////////////

void setup() {
    M5.begin();
    M5.Lcd.setRotation(0);  // 画面向き設定（USB位置基準 0：下/ 1：右/ 2：上/ 3：左）
    M5.Lcd.setTextSize(2);  // 文字サイズ（整数倍率）

    setupDXL();

    uint16_t read_voltage_limit =
            dxl.readControlTableItem(ControlTableItem::MAX_VOLTAGE_LIMIT, DXL_ID);
    M5.Lcd.clear();
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.print(read_voltage_limit);

    delay(2000);
}

void loop() {
    dxl.setGoalPosition(DXL_ID, 1080, UNIT_DEGREE);
    for (byte i = 0; i < 50; i++) {
        float present_deg = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
        M5.Lcd.clear();
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.print(present_deg);
        delay(100);
    }

    dxl.setGoalPosition(DXL_ID, 0, UNIT_DEGREE);
    for (byte i = 0; i < 50; i++) {
        float present_deg = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
        M5.Lcd.clear();
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.print(present_deg);
        delay(100);
    }
}
