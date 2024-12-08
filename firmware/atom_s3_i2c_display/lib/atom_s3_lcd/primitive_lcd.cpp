#include <primitive_lcd.h>

PrimitiveLCD::PrimitiveLCD() : LGFX() {
  init();
  lcdMutex = xSemaphoreCreateMutex();
}

void PrimitiveLCD::drawJpg(const uint8_t *jpg_data, size_t jpg_len, uint16_t x, uint16_t y, uint16_t maxWidth, uint16_t maxHeight, uint16_t offX, uint16_t offY, jpeg_div_t scale) {
  if (lockLcd()) {
    LGFX::drawJpg(jpg_data, jpg_len, x, y, maxWidth, maxHeight, offX, offY, scale);
    unlockLcd();
  }
}

void PrimitiveLCD::qrcode(const char *string, uint16_t x, uint16_t y, uint8_t width, uint8_t version) {
  if (lockLcd()) {
    LGFX::qrcode(string, x, y, width, version);
    unlockLcd();
  }
}

void PrimitiveLCD::printColorText(const String& input) {
  String text = input;
  uint16_t textColor = LGFX::color565(255, 255, 255); // Default text color: white
  uint16_t bgColor = LGFX::color565(0, 0, 0);         // Default background color: black
  int index = 0;
  if (lockLcd()) {
    while (index < text.length()) {
      if (text.charAt(index) == '\x1b' && text.charAt(index + 1) == '[') {
        int mIndex = text.indexOf('m', index);
        if (mIndex != -1) {
          String seq = text.substring(index + 2, mIndex);
          int code = seq.toInt();
          if (seq.startsWith("4")) {
            bgColor = colorMap(code, true);
          } else if (seq.startsWith("3")) {
            textColor = colorMap(code, false);
          }
          text.remove(index, mIndex - index + 1);
          continue;
        }
      }
      LGFX::setTextColor(textColor, bgColor);
      LGFX::print(text.charAt(index));
      index++;
    }
    unlockLcd();
  }
}

void PrimitiveLCD::fillScreen(uint16_t color) {
  if (lockLcd()) {
    LGFX::fillScreen(color);
    unlockLcd();
  }
}

void PrimitiveLCD::fillRect(int x1, int y1, int w, int h, uint16_t color) {
  if (lockLcd()) {
    LGFX::fillRect(x1, y1, w, h, color);
    unlockLcd();
  }
}

void PrimitiveLCD::drawRect(int x1, int y1, int w, int h, uint16_t color) {
  if (lockLcd()) {
    LGFX::drawRect(x1, y1, w, h, color);
    unlockLcd();
  }
}

void PrimitiveLCD::drawLine(int x1, int y1, int x2, int y2, uint16_t color){
  if (lockLcd()) {
    LGFX::drawLine(x1, y1, x2, y2, color);
    unlockLcd();
  }
};

void PrimitiveLCD::drawPixel(int x, int y, uint16_t color){
  if (lockLcd()) {
    LGFX::drawPixel(x, y, color);
    unlockLcd();
  }
};

void PrimitiveLCD::setCursor(int x, int y){
  if (lockLcd()) {
    LGFX::setCursor(x,y);
    unlockLcd();
  }
};

void PrimitiveLCD::setRotation(int lcd_rotation){
  if (lockLcd()) {
    LGFX::setRotation(lcd_rotation);
    unlockLcd();
  }
}

void PrimitiveLCD::clear(){
  if (lockLcd()) {
    LGFX::clear();
    unlockLcd();
  }
}

void PrimitiveLCD::setTextSize(float text_size){
  if (lockLcd()) {
    LGFX::setTextSize(text_size);
    unlockLcd();
  }
}

uint16_t PrimitiveLCD::colorMap(int code, bool isBackground) {
  if (isBackground) {
    switch (code) {
    case 40: return LGFX::color565(0, 0, 0);     // Black
    case 41: return LGFX::color565(255, 0, 0);   // Red
    case 42: return LGFX::color565(0, 255, 0);   // Green
    case 43: return LGFX::color565(255, 255, 0); // Yellow
    case 44: return LGFX::color565(0, 0, 255);   // Blue
    case 45: return LGFX::color565(255, 0, 255); // Magenta
    case 46: return LGFX::color565(0, 255, 255); // Cyan
    case 47: return LGFX::color565(255, 255, 255); // White
    case 49: return LGFX::color565(0, 0, 0);     // Default (Black)
    default: return LGFX::color565(0, 0, 0);
    }
  } else {
    switch (code) {
    case 30: return LGFX::color565(0, 0, 0);     // Black
    case 31: return LGFX::color565(255, 0, 0);   // Red
    case 32: return LGFX::color565(0, 255, 0);   // Green
    case 33: return LGFX::color565(255, 255, 0); // Yellow
    case 34: return LGFX::color565(0, 0, 255);   // Blue
    case 35: return LGFX::color565(255, 0, 255); // Magenta
    case 36: return LGFX::color565(0, 255, 255); // Cyan
    case 37: return LGFX::color565(255, 255, 255); // White
    case 39: return LGFX::color565(255, 255, 255); // Default (White)
    default: return LGFX::color565(255, 255, 255);
    }
  }
}
  
bool PrimitiveLCD::lockLcd() {
  // ミューテックスのロックを試みる（最大100ms待機）
  return xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(100)) == pdTRUE;
}

void PrimitiveLCD::unlockLcd() {
  // ミューテックスを解放
  xSemaphoreGive(lcdMutex);
}
