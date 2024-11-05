#include <display_battery_mode1.h>
// #include "M5AtomS3.h"

// voltage unit: V
class BatteryDisplay
{
public:
  BatteryDisplay(){};
  BatteryDisplay(float bat_cell){bat_cell_ = bat_cell;}
  int getBatCell() {return bat_cell_;}
  void init(){};

  static constexpr float VOLTAGE_100P =  4.2;
  static constexpr float VOLTAGE_90P =  4.085;
  static constexpr float VOLTAGE_80P =  3.999;
  static constexpr float VOLTAGE_70P =  3.936;
  static constexpr float VOLTAGE_60P =  3.883;
  static constexpr float VOLTAGE_50P =  3.839;
  static constexpr float VOLTAGE_40P =  3.812;
  static constexpr float VOLTAGE_30P =  3.791;
  static constexpr float VOLTAGE_20P =  3.747;
  static constexpr float VOLTAGE_10P =  3.1;
  static constexpr float VOLTAGE_0P =  3.0;

private:

  int bat_cell_;
  float battery_voltage_;

  float calcPercentage();
};


inline float BatteryDisplay::calcPercentage()
{
  float average_voltage = battery_voltage_ / bat_cell_;
  // float input_cell = voltage / VOLTAGE_100P;
  float percentage = 0;
  if(average_voltage  > VOLTAGE_90P) percentage = (average_voltage - VOLTAGE_90P) / (VOLTAGE_100P - VOLTAGE_90P) * 10 + 90;
  else if (average_voltage  > VOLTAGE_80P) percentage = (average_voltage - VOLTAGE_80P) / (VOLTAGE_90P - VOLTAGE_80P) * 10 + 80;
  else if (average_voltage  > VOLTAGE_70P) percentage = (average_voltage - VOLTAGE_70P) / (VOLTAGE_80P - VOLTAGE_70P) * 10 + 70;
  else if (average_voltage  > VOLTAGE_60P) percentage = (average_voltage - VOLTAGE_60P) / (VOLTAGE_70P - VOLTAGE_60P) * 10 + 60;
  else if (average_voltage  > VOLTAGE_50P) percentage = (average_voltage - VOLTAGE_50P) / (VOLTAGE_60P - VOLTAGE_50P) * 10 + 50;
  else if (average_voltage  > VOLTAGE_40P) percentage = (average_voltage - VOLTAGE_40P) / (VOLTAGE_50P - VOLTAGE_40P) * 10 + 40;
  else if (average_voltage  > VOLTAGE_30P) percentage = (average_voltage - VOLTAGE_30P) / (VOLTAGE_40P - VOLTAGE_30P) * 10 + 30;
  else if (average_voltage  > VOLTAGE_20P) percentage = (average_voltage - VOLTAGE_20P) / (VOLTAGE_30P - VOLTAGE_20P) * 10 + 20;
  else if (average_voltage  > VOLTAGE_10P) percentage = (average_voltage - VOLTAGE_10P) / (VOLTAGE_20P - VOLTAGE_10P) * 10 + 10;
  else percentage = (average_voltage - VOLTAGE_0P) / (VOLTAGE_10P - VOLTAGE_0P) * 10;
  return percentage;
}

// updateVoltage function to update battery_voltage_

DisplayBatteryMode1* DisplayBatteryMode1::instance = nullptr;

DisplayBatteryMode1::DisplayBatteryMode1(AtomS3LCD &lcd, AtomS3I2C &i2c)
  : atoms3lcd(lcd), atoms3i2c(i2c), Mode("DisplayBatteryMode1") {
    instance = this;
    batDisp_(4);
}

void DisplayBatteryMode1::task(void *parameter) {
  while (true) {
    instance->atoms3i2c.setRequestStr(instance->getModeName());
    // Check for I2C timeout
    if (instance->atoms3i2c.checkTimeout()) {
      instance->atoms3lcd.drawNoDataReceived();
      instance->atoms3lcd.printMessage(instance->getModeName());
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    // Display information
    else {
      instance->atoms3lcd.drawBlack();
      if (instance->atoms3lcd.color_str.isEmpty())
        // instance->atoms3lcd.printColorText("Waiting for " + instance->getModeName());
	batDisp.displaFrame();
      else
        instance->atoms3lcd.printColorText(instance->atoms3lcd.color_str);
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
}

void DisplayBatteryMode1::createTask(uint8_t xCoreID) {
  TaskCreatePinnedToCore(task, "DisplayBatteryMode1", 2048, NULL, 1, &taskHandle, xCoreID);
}

inline void DisplayBatteryMode1::displayFrame()
{
  // Show title.
  instance->atoms3lcd.fillRect(0, 0, LCD_W, 16, MAROON);
  instance->atoms3lcd.setTextSize(2);
  instance->atoms3lcd.setTextColor(WHITE);
  instance->atoms3lcd.drawString("Voltage", 0, 0, 1);

  // Show units.
  instance->atoms3lcd.drawRect(0, 19, LCD_W, 19, YELLOW);
  instance->atoms3lcd.drawLine(LCD_W/2+12, 19, LCD_W/2+12, 37, YELLOW);
  instance->atoms3lcd.drawString("V", LCD_W/2-1, 22, 1);
  instance->atoms3lcd.drawString("%", LCD_W-12, 22, 1);
}

inline void DisplayBatteryMode1::updateVoltage(float voltage)
{
  float voltageRatio = calcPercentage(voltage) / 100;
  voltageRatio = constrain(voltageRatio, 0.0f, 1.0f);

  // Erase screen.
  float LCD_H = instance->atoms3lcd.height()
  float LCD_W = instance->atoms3lcd.width()
  instance->atoms3lcd.fillRect(1, 20, 60, 16, BLACK);
  instance->atoms3lcd.fillRect(LCD_W/2+17, 20, 32, 16, BLACK);

  // Show voltage.
  instance->atoms3lcd.setTextColor(WHITE);
  instance->atoms3lcd.setCursor(2, 21);
  instance->atoms3lcd.printf("%0.2f", battery_voltage_);
  instance->atoms3lcd.drawString("V", LCD_W/2, 22, 1);
  instance->atoms3lcd.setCursor(LCD_W/2+25, 21);
  instance->atoms3lcd.print((uint8_t)(voltageRatio*100));
  instance->atoms3lcd.drawString("%", LCD_W-12, 22, 1);

  // Show Meter
  int32_t rect_x = 0;
  int32_t rect_h = 7;
  int32_t rect_w = LCD_W;
  int32_t radius = 3;
  uint8_t barNum = 10;
  for(byte k = 0; k < barNum; k++)
    {
      int32_t rect_y = LCD_H - rect_h - (rect_h + 2) * k;
      uint16_t color = instance->atoms3lcd.color565(16,16,16);
      if(voltageRatio > float(k+1) / barNum)
        {
          color = instance->atoms3lcd.color565(
                                  (uint8_t)(255 - 255 * (k / float(barNum-1))),
                                  (uint8_t)(255 * (k / float(barNum-1))), 0);
        }
      instance->atoms3lcd.fillRoundRect(rect_x, rect_y, rect_w, rect_h, radius, color);
    }
}
