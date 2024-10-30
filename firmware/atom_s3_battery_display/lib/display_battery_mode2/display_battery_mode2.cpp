#include <display_battery_mode2.h>

DisplayBatteryMode2* DisplayBatteryMode2::instance = nullptr;

DisplayBatteryMode2::DisplayBatteryMode2(AtomS3LCD &lcd, AtomS3I2C &i2c)
  : atoms3lcd(lcd), atoms3i2c(i2c), Mode("DisplayBatteryMode2") {
    instance = this;
}

void DisplayBatteryMode2::task(void *parameter) {
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
        instance->atoms3lcd.printColorText(instance->atoms3lcd.color_str);
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
}

void DisplayBatteryMode2::createTask(uint8_t xCoreID) {
  xTaskCreatePinnedToCore(task, "DisplayBatteryMode2", 2048, NULL, 1, &taskHandle, xCoreID);
}
