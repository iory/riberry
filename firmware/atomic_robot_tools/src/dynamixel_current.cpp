#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <M5AtomS3.h>


// Dynamixel2Arduino ////////////////////////////////////////////////////////
#define DXL_SERIAL Serial1
#define SERIAL_CONFIG SERIAL_8N1

using namespace ControlTableItem;

const long BAUDRATE = 2000000; // 2Mが限界だった
const int TIMEOUT = 100;  //ms
const uint8_t DXL_ID = 0;
const float DXL_PROTOCOL_VERSION = 2.0;

const int EN_PIN = 6;  // Enable Pin
const int RX_PIN = 5;
const int TX_PIN = 38;

Dynamixel2Arduino dxl(DXL_SERIAL, EN_PIN);

void setupDXL()
{
  DXL_SERIAL.begin(BAUDRATE, SERIAL_CONFIG, RX_PIN, TX_PIN, false, TIMEOUT);
  dxl.begin(BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_CURRENT);
  dxl.torqueOn(DXL_ID);

  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 0);
}
/////////////////////////////////////////////////////////////////////////////


void setup() {
  M5.begin();
  M5.Lcd.setRotation(0);  // 画面向き設定（USB位置基準 0：下/ 1：右/ 2：上/ 3：左）
  M5.Lcd.setTextSize(2);  // 文字サイズ（整数倍率）

  setupDXL();

  M5.Lcd.clear();
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.print("Press Button to Plus");
  do { M5.update(); delay(10); } while (!M5.BtnA.isPressed());

  delay(1000);
}


void loop() {
  // dxl.setGoalCurrent(DXL_ID, 10, UNIT_PERCENT);
  dxl.setGoalCurrent(DXL_ID, 500, UNIT_MILLI_AMPERE);
  M5.Lcd.clear();
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.print("Press Button to Minus");
  do { M5.update(); delay(10); } while (!M5.BtnA.isPressed());
  delay(1000);

  dxl.setGoalCurrent(DXL_ID, -800, UNIT_MILLI_AMPERE);
  M5.Lcd.clear();
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.print("Press Button to Plus");
  do { M5.update(); delay(10); } while (!M5.BtnA.isPressed());
  delay(1000);
}