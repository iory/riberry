#ifndef PRIMITIVE_LCD_H
#define PRIMITIVE_LCD_H

#define LGFX_M5ATOMS3
#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>

class PrimitiveLCD : public LGFX {
public:
  PrimitiveLCD();
  void drawJpg(const uint8_t *jpg_data, size_t jpg_len, uint16_t x = 0, uint16_t y = 0, uint16_t maxWidth = 0, uint16_t maxHeight = 0, uint16_t offX = 0, uint16_t offY = 0, jpeg_div_t scale = JPEG_DIV_NONE);
  void qrcode(const char *string, uint16_t x, uint16_t y, uint8_t width, uint8_t version);
  void printColorText(const String& input);
  void fillScreen(uint16_t color);
  void fillRect(int x1, int y1, int w, int h, uint16_t color);
  void drawRect(int x1, int y1, int w, int h, uint16_t color);
  void drawLine(int x1, int y1, int x2, int y2, uint16_t color);
  void drawPixel(int x, int y, uint16_t color);
  void setCursor(int x, int y);
  void setRotation(int lcd_rotation);
  void clear();
  void setTextSize(float text_size);
  /**
   * @brief Convert ANSI color codes to RGB565 values for foreground or background.
   *
   * @param code The ANSI color code.
   * @param isBackground Whether the color is for the background (true) or foreground (false).
   * @return The RGB565 color value.
   */
  uint16_t colorMap(int code, bool isBackground);

private:
  SemaphoreHandle_t lcdMutex;
  bool lockLcd();
  void unlockLcd();
};

#endif // PRIMITIVE_LCD_H
