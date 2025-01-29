#include <display_odom_mode.h>

DisplayOdomMode::DisplayOdomMode() : Mode("DisplayOdomMode") {}

void DisplayOdomMode::task(PrimitiveLCD &lcd, CommunicationBase &com) {
    while (true) {
        // Check for I2C timeout
        if (com.checkTimeout()) {
            lcd.drawNoDataReceived();
            lcd.printColorText(getModeName() + "\n");
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        // Display information
        else {
            lcd.drawBlack();
            if (lcd.color_str.isEmpty())
                lcd.printColorText("Waiting for " + getModeName());
            else
                lcd.printColorText(lcd.color_str);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}
