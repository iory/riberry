#include <display_qrcode_mode.h>

DisplayQRcodeMode* DisplayQRcodeMode::instance = nullptr;

DisplayQRcodeMode::DisplayQRcodeMode(PrimitiveLCD &lcd, CommunicationBase &i2c)
  : lcd(lcd), comm(i2c), Mode("DisplayQRcodeMode") {
    instance = this;
}

void DisplayQRcodeMode::task(void *parameter) {
  String previousQrCodeData;
  while (true) {
    instance->comm.setRequestStr(instance->getModeName());
    // Check for I2C timeout
    if (instance->comm.checkTimeout()) {
      instance->lcd.drawNoDataReceived();
      instance->lcd.printColorText(instance->getModeName() + "\n");
      vTaskDelay(pdMS_TO_TICKS(500));
      previousQrCodeData = "";
      continue;
    }
    // Display QR code
    else {
      if (instance->lcd.qrCodeData.isEmpty()) {
        instance->lcd.drawBlack();
        instance->lcd.printColorText("Waiting for " + instance->getModeName());
        previousQrCodeData = "";
      } else {
        if (previousQrCodeData.equals(instance->lcd.qrCodeData)) {
          vTaskDelay(pdMS_TO_TICKS(10));
        } else {
          instance->lcd.drawBlack();
          instance->lcd.drawQRcode(instance->lcd.qrCodeData);
          previousQrCodeData = instance->lcd.qrCodeData;
        }
      }
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
}

void DisplayQRcodeMode::createTask(uint8_t xCoreID) {
  xTaskCreatePinnedToCore(task, "Display QRcode Mode", 2048, NULL, 1, &taskHandle, xCoreID);
}
