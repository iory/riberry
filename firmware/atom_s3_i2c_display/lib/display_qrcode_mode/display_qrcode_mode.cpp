#include <display_qrcode_mode.h>

DisplayQRcodeMode* DisplayQRcodeMode::instance = nullptr;

DisplayQRcodeMode::DisplayQRcodeMode(PrimitiveLCD &lcd, CommunicationBase &i2c)
  : lcd(lcd), comm(i2c), Mode("DisplayQRcodeMode") {
    instance = this;
}

void DisplayQRcodeMode::task(void *parameter) {
  while (true) {
    instance->comm.setRequestStr(instance->getModeName());
    // Check for I2C timeout
    if (instance->comm.checkTimeout()) {
      instance->lcd.drawNoDataReceived();
      instance->lcd.printColorText(instance->getModeName() + "\n");
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    // Display QR code
    else {
      instance->lcd.drawBlack();
      if (instance->lcd.qrCodeData.isEmpty())
        instance->lcd.printColorText("Waiting for " + instance->getModeName());
      else
        instance->lcd.drawQRcode(instance->lcd.qrCodeData);
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
}

void DisplayQRcodeMode::createTask(uint8_t xCoreID) {
  xTaskCreatePinnedToCore(task, "Display QRcode Mode", 2048, NULL, 1, &taskHandle, xCoreID);
}
