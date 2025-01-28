#include <display_qrcode_mode.h>

DisplayQRcodeMode::DisplayQRcodeMode()
  : Mode("DisplayQRcodeMode") {
}

void DisplayQRcodeMode::task(PrimitiveLCD &lcd, CommunicationBase &i2c) {
  String previousQrCodeData;
  while (true) {
    i2c.setRequestStr(getModeName());
    // Check for I2C timeout
    if (i2c.checkTimeout()) {
      lcd.drawNoDataReceived();
      lcd.printColorText(getModeName() + "\n");
      vTaskDelay(pdMS_TO_TICKS(500));
      previousQrCodeData = "";
      continue;
    }
    // Display QR code
    else {
      if (lcd.qrCodeData.isEmpty()) {
        lcd.drawBlack();
        lcd.printColorText("Waiting for " + getModeName());
        previousQrCodeData = "";
      } else {
        if (previousQrCodeData.equals(lcd.qrCodeData)) {
          vTaskDelay(pdMS_TO_TICKS(10));
        } else {
          lcd.drawBlack();
          lcd.drawQRcode(lcd.qrCodeData);
          previousQrCodeData = lcd.qrCodeData;
        }
      }
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
}