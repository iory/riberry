#include <display_image_mode.h>

DisplayImageMode* DisplayImageMode::instance = nullptr;

DisplayImageMode::DisplayImageMode(AtomS3LCD &lcd, AtomS3I2C &i2c)
  : atoms3lcd(lcd), atoms3i2c(i2c), Mode("DisplayImageMode") {
    instance = this;
}

void DisplayImageMode::task(void *parameter) {
  while (true) {
    instance->atoms3i2c.setRequestStr(instance->getModeName());
    // Check for I2C timeout
    if (instance->atoms3i2c.checkTimeout()) {
      instance->atoms3lcd.drawNoDataReceived();
      instance->atoms3lcd.printColorText(instance->getModeName() + "\n");
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    // Display Image code
    else {
      if (instance->atoms3lcd.mode_changed) {
        instance->atoms3lcd.drawBlack();
        instance->atoms3lcd.printColorText("Waiting for " + instance->getModeName() + ". \x1b[31mrosparam set \x1b[32m/display_image <Your Image Topic>\x1b[39m");
        instance->atoms3lcd.mode_changed = false;
      }
      if (instance->atoms3lcd.readyJpeg == true) {
        instance->atoms3lcd.drawImage(instance->atoms3lcd.jpegBuf, instance->atoms3lcd.jpegLength);
        instance->atoms3lcd.readyJpeg = false;
      }
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}

void DisplayImageMode::createTask(uint8_t xCoreID) {
  xTaskCreatePinnedToCore(task, "Display Image Mode", 2048, NULL, 1, &taskHandle, xCoreID);
}
