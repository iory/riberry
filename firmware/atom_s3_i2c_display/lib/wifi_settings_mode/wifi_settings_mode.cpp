#include <wifi_settings_mode.h>

#include "string_utils.h"

WiFiSettingsMode::WiFiSettingsMode(ButtonManager &button_manager)
    : Mode(ModeType::WIFI_SETTINGS), button_manager(button_manager) {}

void WiFiSettingsMode::task(PrimitiveLCD &lcd, CommunicationBase &com) {
    prevStr = "";
    unsigned long prevTime = millis();
    ButtonState buttonState;
    ButtonState previousButtonState = NOT_CHANGED;
    unsigned long startTime = 0;
    while (running) {
        if (handleTimeout(lcd, com)) continue;
        if (millis() - prevTime >= 2000) {
            com.setAdditionalRequestBytes((uint8_t *)"", 0);
        }

        buttonState = button_manager.getButtonState();
        if (previousButtonState != buttonState) {
            previousButtonState = buttonState;
            if (buttonState == SINGLE_CLICK) {
                com.setAdditionalRequestBytes((uint8_t *)"wifi_connect", 12);
                prevTime = millis();
                prevStr = "";
            }
        }
        if (!lcd.color_str.isEmpty() && !compareIgnoringEscapeSequences(prevStr, lcd.color_str)) {
            lcd.drawBlack();
            prevStr = lcd.color_str;
            lcd.printColorText(lcd.color_str);
            lcd.color_str = "";
        } else if (!lcd.qrCodeData.isEmpty() &&
                   !compareIgnoringEscapeSequences(prevStr, lcd.qrCodeData)) {
            lcd.drawBlack();
            prevStr = lcd.qrCodeData;
            lcd.drawQRcode(lcd.qrCodeData);
            lcd.qrCodeData = "";
        }
        delayWithTimeTracking(pdMS_TO_TICKS(100));
    }
}
