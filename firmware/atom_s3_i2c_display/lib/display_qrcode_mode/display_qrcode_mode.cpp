#include <display_qrcode_mode.h>

DisplayQRcodeMode* DisplayQRcodeMode::instance = nullptr;

DisplayQRcodeMode::DisplayQRcodeMode(AtomS3LCD &lcd, AtomS3I2C &i2c)
  : atoms3lcd(lcd), atoms3i2c(i2c), Mode("DisplayQRcodeMode") {
    instance = this;
}

void DisplayQRcodeMode::task(void *parameter) {
  while (true) {
    instance->atoms3i2c.setRequestStr(instance->getModeName());
    // Check for I2C timeout
    if (instance->atoms3i2c.checkTimeout()) {
      instance->atoms3lcd.drawNoDataReceived();
      instance->atoms3lcd.printColorText(instance->getModeName() + "\n");
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    // Display QR code
    else {
      instance->atoms3lcd.drawBlack();
      if (instance->atoms3lcd.qrCodeData.isEmpty())
        instance->atoms3lcd.printColorText("Waiting for " + instance->getModeName());
      else
        instance->atoms3lcd.drawQRcode(instance->atoms3lcd.qrCodeData);
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
}

void DisplayQRcodeMode::createTask(uint8_t xCoreID) {
  xTaskCreatePinnedToCore(task, "Display QRcode Mode", 2048, NULL, 1, &taskHandle, xCoreID);
}
