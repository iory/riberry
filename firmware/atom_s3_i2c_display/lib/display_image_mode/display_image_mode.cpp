#include <display_image_mode.h>

DisplayImageMode::DisplayImageMode() : Mode("DisplayImageMode") {}

void DisplayImageMode::task(PrimitiveLCD &lcd, CommunicationBase &com) {
    while (true) {
        com.setRequestStr(getModeName());
        // Check for I2C timeout
        if (com.checkTimeout()) {
            lcd.drawNoDataReceived();
            lcd.printColorText(getModeName() + "\n");
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        // Display Image code
        else {
            if (lcd.mode_changed) {
                lcd.drawBlack();
                lcd.printColorText("Waiting for " + getModeName() +
                                   ". \x1b[31mrosparam set \x1b[32m/display_image <Your "
                                   "Image Topic>\x1b[39m");
                lcd.mode_changed = false;
            }
            if (lcd.readyJpeg == true) {
                lcd.drawImage(lcd.jpegBuf, lcd.jpegLength);
                lcd.readyJpeg = false;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}