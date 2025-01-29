#include <display_qrcode_mode.h>

DisplayQRcodeMode::DisplayQRcodeMode() : Mode("DisplayQRcodeMode") {}

void DisplayQRcodeMode::task(PrimitiveLCD &lcd, CommunicationBase &com) {
    while (true) {
        if (handleTimeout(lcd, com) || handleEmptyDisplay(lcd)) continue;

        if (prevStr.equals(lcd.qrCodeData)) {
            vTaskDelay(pdMS_TO_TICKS(10));
        } else {
            lcd.drawBlack();
            lcd.drawQRcode(lcd.qrCodeData);
            prevStr = lcd.qrCodeData;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}