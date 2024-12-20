#include <display_battery_graph_mode.h>

DisplayBatteryGraphMode* DisplayBatteryGraphMode::instance = nullptr;

DisplayBatteryGraphMode::DisplayBatteryGraphMode(AtomS3LCD &lcd, AtomS3I2C &i2c)
  : atoms3lcd(lcd), atoms3i2c(i2c), Mode("DisplayBatteryGraphMode") {
    instance = this;
}

void DisplayBatteryGraphMode::task(void *parameter) {
  while (true) {
    instance->atoms3i2c.setRequestStr(instance->getModeName());
    // Check for I2C timeout
    if (instance->atoms3i2c.checkTimeout()) {
      instance->atoms3lcd.drawNoDataReceived();
      instance->atoms3lcd.printColorText(instance->getModeName() + "\n");
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    // Display information
    else {
      unsigned long currentTime = millis();
      if (instance->atoms3lcd.color_str.isEmpty()) {
        instance->atoms3lcd.drawBlack();
        instance->atoms3lcd.printColorText("Waiting for " + instance->getModeName());
      }else {
        // Split by comma
        char* parts[max_buffer_length + 3];
        int numParts = instance->atoms3i2c.splitString(instance->atoms3lcd.color_str,
                                                       ',', parts, max_buffer_length + 3);

        uint new_charge_status = 0;
        if (numParts > 0)
          new_charge_status = atoi(parts[0]);

        String charge_current = "";
        if (numParts > 1)
          charge_current = String(parts[1]);

        int duration = 0;
        if (numParts > 2)
          duration = atoi(parts[2]); // Convert string to integer

        float percentages[max_buffer_length];
        for (int i = 3; i < numParts && i - 3 < max_buffer_length; ++i) {
          percentages[i - 3] = atof(parts[i]); // Convert string to float
        }

        // Not redraw until 10 seconds have passed
        bool skipDrawing = (new_charge_status == instance->charge_status) &&
          (currentTime - instance->atoms3lcd.getLastDrawTime() < 10000);
        if (skipDrawing) {
          vTaskDelay(pdMS_TO_TICKS(1000));
          continue;
        }

        if (numParts > 0) {
          instance->updateGraph(percentages, numParts-3, new_charge_status, charge_current, duration);
        }
        instance->atoms3lcd.setLastDrawTime(currentTime);
      }
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
}

void DisplayBatteryGraphMode::createTask(uint8_t xCoreID) {
  xTaskCreatePinnedToCore(task, "Display Battery Graph Mode", 2048, NULL, 1, &taskHandle, xCoreID);
}

void DisplayBatteryGraphMode::updateGraph(float* buffer, int buffer_length,
                                          uint new_charge_status, String current, int duration) {
  int gap = 1; // width between bar to bar
  int buffer_w = (graph_w - (buffer_length - 1) * gap) / buffer_length;

  instance->atoms3lcd.drawBlack();
  drawBatteryIcon(0, 0, 35, 20, (int)buffer[buffer_length-1]);
  // top text
  String line1 = "";
  String line2 = "";
  if (new_charge_status == 0) {
    line1 = "\x1b[31mNo";
    line2 = "\x1b[31mcharging";
  }else if (new_charge_status == 1) {
    line1 = "\x1b[33mTrickle";
    line2 = " " + current + "mA";
  }else if (new_charge_status == 2) {
    line1 = String("\x1b[33m") + "Pre-charge";
    line2 = " " + current + "mA";
  }else if (new_charge_status == 3) {
    line1 = String("\x1b[32m") + "CC charge";
    line2 = " " + current + "mA";
  }else if (new_charge_status == 4) {
    line1 = String("\x1b[32m") + "CV charge";
    line2 = " " + current + "mA";
  }else if (new_charge_status == 5) {
    line1 = "\x1b[34mCharge";
    line2 = "\x1b[34mtermination";
  }
  instance->atoms3lcd.setTextSize(1.4);
  instance->atoms3lcd.setCursor(41, 0);
  instance->atoms3lcd.printColorText(line1);
  instance->atoms3lcd.setCursor(41, 14);
  instance->atoms3lcd.printColorText(line2);
  instance->atoms3lcd.drawLine(0, title_h-9, LCD_W, title_h-9, TFT_WHITE );
  // y label text
  instance->atoms3lcd.setTextSize(1);
  instance->atoms3lcd.setCursor(0, title_h);
  instance->atoms3lcd.printColorText("100");
  instance->atoms3lcd.setCursor(0, title_h + graph_h/2);
  instance->atoms3lcd.printColorText(" 50");
  instance->atoms3lcd.setCursor(0, title_h + graph_h);
  instance->atoms3lcd.printColorText("  0");
  // y axis line
  instance->atoms3lcd.drawLine(y_label_w, title_h, y_label_w, LCD_H-(x_label_h+2)-1, TFT_WHITE );
  // bar graph
  for (int i = 0; i < buffer_length; i++) {
    if (buffer[i] == 0)
      continue;
    int scaled_percentage = buffer[i] * (graph_h-1) / 100;
    for (int j = 0; j <= scaled_percentage; j++) {
      uint16_t color = calculateColor(j * 100 / (graph_h-1));
      instance->atoms3lcd.fillRect((y_label_w+y_line_w)+i*(buffer_w+gap), LCD_H-(x_label_h+2)-j-1,
                                   buffer_w, 1, color);
    }
  }
  // x label text
  String x_label = "duration:" + String(duration) + "[s]";
  instance->atoms3lcd.setCursor(LCD_W/2-x_label.length()*5/2, LCD_H-x_label_h);
  instance->atoms3lcd.printColorText(x_label);
  instance->atoms3lcd.setTextSize(1.5); // Restore default size for other modes

  instance->charge_status = new_charge_status;
}

// Gradation from red to yellow to green
uint16_t DisplayBatteryGraphMode::calculateColor(float percentage) {
  uint8_t red, green;
  if (percentage <= 50.0f) {
    red = 255;
    green = (uint8_t)(percentage * 2.55 * 2);
  } else {
    red = (uint8_t)((100 - percentage) * 2.55 * 2);
    green = 255;
  }
  return instance->atoms3lcd.color565(red, green, 0);
}

void DisplayBatteryGraphMode::drawBatteryIcon(int x, int y, int width, int height, int batteryLevel) {
  int borderWidth = 2;
  int tipWidth = height / 4;
  int tipHeight = height / 2;

  // Drawing the outer frame
  instance->atoms3lcd.drawRect(x, y, width - tipWidth, height, TFT_WHITE );
  // Drawing the tip
  instance->atoms3lcd.fillRect(x + width - tipWidth, y + height / 4, tipWidth, tipHeight, TFT_WHITE );
  // Filling in the contents with white
  instance->atoms3lcd.fillRect(x + borderWidth, y + borderWidth,
                               width - tipWidth - borderWidth * 2, height - borderWidth * 2, TFT_WHITE );

  // Displaying remaining volume in black
  String percentage = "\x1b[30m\x1b[47m" + String(batteryLevel);
  int textX = (batteryLevel >= 100) ? (x + 3) : (x + 7);
  int textY = y + height / 4;
  instance->atoms3lcd.setTextSize(1.5);
  instance->atoms3lcd.setCursor(textX, textY);
  instance->atoms3lcd.printColorText(percentage);
}
