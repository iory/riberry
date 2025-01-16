#include <display_information_mode.h>

DisplayInformationMode* DisplayInformationMode::instance = nullptr;

DisplayInformationMode::DisplayInformationMode(PrimitiveLCD &lcd, CommunicationBase &i2c)
  : lcd(lcd), comm(i2c), Mode("DisplayInformationMode") {
    instance = this;
}

void DisplayInformationMode::task(void *parameter) {
  String prevStr;
  while (true) {
    instance->comm.setRequestStr(instance->getModeName());
    // Check for I2C timeout
    if (instance->comm.checkTimeout()) {
      instance->lcd.drawNoDataReceived();
      instance->lcd.printColorText(instance->getModeName() + "\n");
      vTaskDelay(pdMS_TO_TICKS(500));
      prevStr = "";
      continue;
    }
    // Display information
    else {
      if (instance->lcd.color_str.isEmpty()) {
        instance->lcd.drawBlack();
        instance->lcd.printColorText("Waiting for " + instance->getModeName());
        prevStr = "";
      } else {
        if (prevStr.equals(instance->lcd.color_str)) {
          vTaskDelay(pdMS_TO_TICKS(10));
        } else {
          instance->lcd.drawBlack();
          prevStr = instance->lcd.color_str;
          instance->lcd.printColorText(instance->lcd.color_str);
        }
      }
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
}

void DisplayInformationMode::createTask(uint8_t xCoreID) {
  xTaskCreatePinnedToCore(task, "Display Information Mode", 2048, NULL, 1, &taskHandle, xCoreID);
}
