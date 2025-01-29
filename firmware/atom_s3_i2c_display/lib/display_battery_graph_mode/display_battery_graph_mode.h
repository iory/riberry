#ifndef ATOM_S3_DISPLAY_BATTERY_GRAPH_MODE_H
#define ATOM_S3_DISPLAY_BATTERY_GRAPH_MODE_H

#include <mode.h>
#include <primitive_lcd.h>
#include <communication_base.h>

class DisplayBatteryGraphMode : public Mode {
public:
  DisplayBatteryGraphMode();

private:
  const int32_t title_h = 35;
  const int32_t y_label_w = 18;
  const int32_t y_line_w = 1;
  const int32_t x_label_h = 8;
  int32_t graph_h;  //83 for atom s3
  int32_t graph_w; //109 for atom s3
  // これを100など大きくしすぎるとプログラムが落ちる
  const int max_buffer_length = 20;
  uint charge_status;

  void task(PrimitiveLCD &lcd, CommunicationBase &com) override;
  void updateGraph(float* buffer, int buffer_length, uint new_charge_status, String current, int duration, PrimitiveLCD &lcd);
  uint16_t calculateColor(float percentage, PrimitiveLCD &lcd);
  void drawBatteryIcon(int x, int y, int width, int height, int batteryLevel, PrimitiveLCD &lcd);
};

#endif // DISPLAY_BATTERY_GRAPH_MODE_H
