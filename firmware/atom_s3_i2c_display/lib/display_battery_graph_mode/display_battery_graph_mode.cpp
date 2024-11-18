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
      instance->atoms3lcd.printMessage(instance->getModeName());
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    // Display information
    else {
      instance->atoms3lcd.drawBlack();
      if (instance->atoms3lcd.color_str.isEmpty())
        instance->atoms3lcd.printColorText("Waiting for " + instance->getModeName());
      else {
        int index = 0;
        // Split by comma
        char* token = strtok(const_cast<char*>(instance->atoms3lcd.color_str.c_str()), ",");
        int duration = 0;
        if (token != NULL) {
          duration = atoi(token); // Convert string to integer
          token = strtok(NULL, ",");
        }

        float percentages[max_buffer_length]; // Receive less than 100 floats
        while (token != NULL && index < max_buffer_length) {
          percentages[index] = atof(token); // Convert string to float
          token = strtok(NULL, ",");
          index++;
        }
        if (index > 0) {
          instance->updateGraph(percentages, index, duration);
        }
      }
      vTaskDelay(pdMS_TO_TICKS(10000));
    }
  }
}

void DisplayBatteryGraphMode::createTask(uint8_t xCoreID) {
  xTaskCreatePinnedToCore(task, "Display Battery Graph Mode", 2048, NULL, 1, &taskHandle, xCoreID);
}

void DisplayBatteryGraphMode::updateGraph(float* buffer, int buffer_length, int duration) {
  int gap = 1; // width between bar to bar
  int buffer_w = (graph_w - (buffer_length - 1) * gap) / buffer_length;

  instance->atoms3lcd.drawBlack();
  // top text
  instance->atoms3lcd.setTextSize(2);
  instance->atoms3lcd.printColorText("Bat " + String(buffer[buffer_length-1]) + "%");
  // y label text
  instance->atoms3lcd.setTextSize(1);
  instance->atoms3lcd.setCursor(0, title_h);
  instance->atoms3lcd.printColorText("100");
  instance->atoms3lcd.setCursor(0, title_h + graph_h/2);
  instance->atoms3lcd.printColorText(" 50");
  instance->atoms3lcd.setCursor(0, title_h + graph_h);
  instance->atoms3lcd.printColorText("  0");
  // y axis line
  instance->atoms3lcd.drawLine(y_label_w, title_h, y_label_w, LCD_H-(x_label_h+2)-1, 0xffff);
  // bar graph
  for (int i = 0; i < buffer_length; i++) {
    if (buffer[i] == 0)
      continue;
    int scaled_percentage = buffer[i] * (graph_h-1) / 100;
    for (int j = 0; j <= scaled_percentage; j++) {
      uint16_t color = calculateColor(j * 100 / (graph_h-1));
      instance->atoms3lcd.fillRect((y_label_w+y_line_w)+i*(buffer_w+gap), LCD_H-(x_label_h+2)-j-1, buffer_w, 1, color);
    }
  }
  // x label text
  String x_label = "duration:" + String(duration) + "[s]";
  instance->atoms3lcd.setCursor(LCD_W/2-x_label.length()*5/2, LCD_H-x_label_h);
  instance->atoms3lcd.printColorText(x_label);
  instance->atoms3lcd.setTextSize(1.5); // Restore default size for other modes
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
