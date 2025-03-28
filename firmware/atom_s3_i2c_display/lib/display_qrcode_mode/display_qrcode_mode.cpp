#include <display_qrcode_mode.h>

DisplayQRcodeMode::DisplayQRcodeMode(bool isSkippable)
    : Mode(ModeType::DISPLAY_QRCODE, isSkippable) {}

void DisplayQRcodeMode::task(PrimitiveLCD &lcd, CommunicationBase &com) {
    while (running) {
        if (handleTimeout(lcd, com)) continue;
        if (lcd.qrCodeData.isEmpty()) {
            String waitStr = "Waiting for " + getName();
            lcd.drawBlack();
            lcd.printColorText(waitStr);
            delayWithTimeTracking(pdMS_TO_TICKS(500));
            continue;
        }
        if (prevStr.equals(lcd.qrCodeData)) {
            delayWithTimeTracking(pdMS_TO_TICKS(10));
        } else {
            lcd.drawBlack();
            lcd.drawQRcode(lcd.qrCodeData);
            prevStr = lcd.qrCodeData;
        }
        delayWithTimeTracking(pdMS_TO_TICKS(1000));
    }
}
