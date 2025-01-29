#include <display_information_mode.h>
#include <string_utils.h>

DisplayInformationMode::DisplayInformationMode()
  : Mode("DisplayInformationMode") {
}

void DisplayInformationMode::task(PrimitiveLCD &lcd, CommunicationBase &com) {
  String prevStr;
  while (true) {
    com.setRequestStr(getModeName());
    // Check for I2C timeout
    if (com.checkTimeout()) {
      lcd.drawNoDataReceived();
      lcd.printColorText(getModeName() + "\n");
      vTaskDelay(pdMS_TO_TICKS(500));
      prevStr = "";
      continue;
    }
    // Display information
    else {
      if (lcd.color_str.isEmpty()) {
        lcd.drawBlack();
        lcd.printColorText("Waiting for " + getModeName());
        prevStr = "";
      } else {
        // The reason for using this here is that the equals function relies on strcmp,
        // which cannot properly handle escape sequences. As a result, it may fail to compare strings correctly.
        if (compareIgnoringEscapeSequences(prevStr, lcd.color_str)) {
          vTaskDelay(pdMS_TO_TICKS(10));
        } else {
          lcd.drawBlack();
          prevStr = lcd.color_str;
          lcd.printColorText(lcd.color_str);
        }
      }
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
}