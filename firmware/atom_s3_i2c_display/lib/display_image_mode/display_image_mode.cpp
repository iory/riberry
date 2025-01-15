#include <display_image_mode.h>

DisplayImageMode* DisplayImageMode::instance = nullptr;

DisplayImageMode::DisplayImageMode(PrimitiveLCD &lcd, CommunicationBase &i2c)
  : lcd(lcd), comm(i2c), Mode("DisplayImageMode") {
    instance = this;
}

void DisplayImageMode::task(void *parameter) {
  while (true) {
    instance->comm.setRequestStr(instance->getModeName());
    // Check for I2C timeout
    if (instance->comm.checkTimeout()) {
      instance->lcd.drawNoDataReceived();
      instance->lcd.printColorText(instance->getModeName() + "\n");
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    // Display Image code
    else {
      if (instance->lcd.mode_changed) {
        instance->lcd.drawBlack();
        instance->lcd.printColorText("Waiting for " + instance->getModeName() + ". \x1b[31mrosparam set \x1b[32m/display_image <Your Image Topic>\x1b[39m");
        instance->lcd.mode_changed = false;
      }
      if (instance->lcd.readyJpeg == true) {
        instance->lcd.drawImage(instance->lcd.jpegBuf, instance->lcd.jpegLength);
        instance->lcd.readyJpeg = false;
      }
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}

void DisplayImageMode::createTask(uint8_t xCoreID) {
  xTaskCreatePinnedToCore(task, "Display Image Mode", 2048, NULL, 1, &taskHandle, xCoreID);
}
