#include <display_odom_mode.h>

DisplayOdomMode* DisplayOdomMode::instance = nullptr;

DisplayOdomMode::DisplayOdomMode(PrimitiveLCD &lcd, CommunicationBase &i2c)
  : lcd(lcd), comm(i2c), Mode("DisplayOdomMode") {
    instance = this;
}

void DisplayOdomMode::task(void *parameter) {
  while (true) {
    instance->comm.setRequestStr(instance->getModeName());
    // Check for I2C timeout
    if (instance->comm.checkTimeout()) {
      instance->lcd.drawNoDataReceived();
      instance->lcd.printColorText(instance->getModeName() + "\n");
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    // Display information
    else {
      instance->lcd.drawBlack();
      if (instance->lcd.color_str.isEmpty())
        instance->lcd.printColorText("Waiting for " + instance->getModeName());
      else
        instance->lcd.printColorText(instance->lcd.color_str);
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
}

void DisplayOdomMode::createTask(uint8_t xCoreID) {
  xTaskCreatePinnedToCore(task, "DisplayOdomMode", 2048, NULL, 1, &taskHandle, xCoreID);
}
