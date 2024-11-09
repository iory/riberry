#include <display_battery_mode.h>
// #include "M5AtomS3.h"

DisplayBatteryMode* DisplayBatteryMode::instance = nullptr;

DisplayBatteryMode::DisplayBatteryMode(AtomS3LCD &lcd, AtomS3I2C &i2c)
  : atoms3lcd(lcd), atoms3i2c(i2c), Mode("DisplayBatteryMode"), batDisp_(4) {
    instance = this;
}

void DisplayBatteryMode::task(void *parameter) {
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
	  // instance->displayFrame();
	  // instance->atoms3lcd.printColorText(instance->atoms3lcd.color_str);
	}
        // instance->atoms3lcd.printColorText(instance->atoms3lcd.color_str);
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
}

void DisplayBatteryMode::createTask(uint8_t xCoreID) {
  xTaskCreatePinnedToCore(task, "DisplayBatteryMode", 2048, NULL, 1, &taskHandle, xCoreID);
}

inline void DisplayBatteryMode::displayFrame()
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

inline void DisplayBatteryMode::updateVoltage(float voltage)
{
  float voltageRatio = batDisp_.calcPercentage(voltage) / 100;
  voltageRatio = constrain(voltageRatio, 0.0f, 1.0f);

  // Erase screen.
  instance->atoms3lcd.fillRect(1, 20, 60, 16, BLACK);
  instance->atoms3lcd.fillRect(LCD_W/2+17, 20, 32, 16, BLACK);

  // Show voltage.
  instance->atoms3lcd.setTextColor(WHITE); //OK
  instance->atoms3lcd.setCursor(2, 21); //OK
  // instance->atoms3lcd.printf("%.2f", voltage); // NG
  String voltage_str = String(voltage, 2);
  instance->atoms3lcd.printMessage(voltage_str); //OK
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
      uint16_t color = instance->atoms3lcd.color565(16,16,16); // OK
      if(voltageRatio > float(k+1) / barNum)
        {
          color = instance->atoms3lcd.color565(
                                  (uint8_t)(255 - 255 * (k / float(barNum-1))),
                                  (uint8_t)(255 * (k / float(barNum-1))), 0);
        }
      instance->atoms3lcd.fillRoundRect(rect_x, rect_y, rect_w, rect_h, radius, color);
    }
}