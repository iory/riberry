#include <display_battery_mode1.h>
// #include "M5AtomS3.h"

DisplayBatteryMode1* DisplayBatteryMode1::instance = nullptr;

DisplayBatteryMode1::DisplayBatteryMode1(AtomS3LCD &lcd, AtomS3I2C &i2c)
  : atoms3lcd(lcd), atoms3i2c(i2c), Mode("DisplayBatteryMode1"), batDisp_(4) {
    instance = this;
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
        instance->atoms3lcd.printColorText("Waiting for " + instance->getModeName());
      else
	{
	  float voltage = std::stof(std::string(instance->atoms3lcd.color_str.c_str()));
	  instance->updateVoltage(voltage);
	  instance->displayFrame();
	}
        // instance->atoms3lcd.printColorText(instance->atoms3lcd.color_str);
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
  float voltageRatio = batDisp_.calcPercentage(voltage) / 100;
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
