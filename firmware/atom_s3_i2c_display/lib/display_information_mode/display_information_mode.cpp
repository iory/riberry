#include <display_information_mode.h>

DisplayInformationMode* DisplayInformationMode::instance = nullptr;

DisplayInformationMode::DisplayInformationMode(AtomS3LCD &lcd, CommunicationBase &i2c)
  : atoms3lcd(lcd), comm(i2c), Mode("DisplayInformationMode") {
    instance = this;
}

void DisplayInformationMode::task(void *parameter) {
  while (true) {
    instance->comm.setRequestStr(instance->getModeName());
    // Check for I2C timeout
    if (instance->comm.checkTimeout()) {
      instance->atoms3lcd.drawNoDataReceived();
      instance->atoms3lcd.printColorText(instance->getModeName() + "\n");
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

void DisplayInformationMode::createTask(uint8_t xCoreID) {
  xTaskCreatePinnedToCore(task, "Display Information Mode", 2048, NULL, 1, &taskHandle, xCoreID);
}
