#ifndef ATOM_S3_DISPLAY_BATTERY_GRAPH_MODE_H
#define ATOM_S3_DISPLAY_BATTERY_GRAPH_MODE_H

#include <mode.h>
#include <atom_s3_lcd.h>
#include <atom_s3_i2c.h>

class DisplayBatteryGraphMode : public Mode {
public:
  DisplayBatteryGraphMode(AtomS3LCD &lcd, AtomS3I2C &i2c);
  void createTask(uint8_t xCoreID) override;

private:
  static const int32_t title_h = 35;
  static const int32_t y_label_w = 18;
  static const int32_t y_line_w = 1;
  static const int32_t x_label_h = 8;
  static const int32_t graph_h = LCD_H - title_h - x_label_h - 2; //83
  static const int32_t graph_w = LCD_W - y_label_w - y_line_w; //109
  static DisplayBatteryGraphMode* instance;
  static const int max_buffer_length = 100;
  String charge_status;
  AtomS3LCD &atoms3lcd;
  AtomS3I2C &atoms3i2c;

  static void task(void *parameter);
  void updateGraph(float* buffer, int buffer_length, String status, String current, int duration);
  uint16_t calculateColor(float percentage);
  void drawBatteryIcon(int x, int y, int width, int height, int batteryLevel);
};

#endif // DISPLAY_BATTERY_GRAPH_MODE_H
