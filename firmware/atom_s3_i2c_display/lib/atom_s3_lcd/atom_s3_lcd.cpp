#include <atom_s3_lcd.h>

AtomS3LCD::AtomS3LCD()
  : lcd(), qrCodeData("") {
  init();
}

void AtomS3LCD::init() {
    lcd.init();
    lcd.setRotation(lcd_rotation);
    lcd.clear();
    lcd.setTextSize(1.5);
}

void AtomS3LCD::clear() {
    lcd.clear();
}

void AtomS3LCD::printMessage(const String& message) {
    lcd.println(message);
}

uint16_t AtomS3LCD::colorMap(int code, bool isBackground) {
    if (isBackground) {
        switch (code) {
        case 40: return lcd.color565(0, 0, 0);     // Black
        case 41: return lcd.color565(255, 0, 0);   // Red
        case 42: return lcd.color565(0, 255, 0);   // Green
        case 43: return lcd.color565(255, 255, 0); // Yellow
        case 44: return lcd.color565(0, 0, 255);   // Blue
        case 45: return lcd.color565(255, 0, 255); // Magenta
        case 46: return lcd.color565(0, 255, 255); // Cyan
        case 47: return lcd.color565(255, 255, 255); // White
        case 49: return lcd.color565(0, 0, 0);     // Default (Black)
        default: return lcd.color565(0, 0, 0);
        }
    } else {
        switch (code) {
        case 30: return lcd.color565(0, 0, 0);     // Black
        case 31: return lcd.color565(255, 0, 0);   // Red
        case 32: return lcd.color565(0, 255, 0);   // Green
        case 33: return lcd.color565(255, 255, 0); // Yellow
        case 34: return lcd.color565(0, 0, 255);   // Blue
        case 35: return lcd.color565(255, 0, 255); // Magenta
        case 36: return lcd.color565(0, 255, 255); // Cyan
        case 37: return lcd.color565(255, 255, 255); // White
        case 39: return lcd.color565(255, 255, 255); // Default (White)
        default: return lcd.color565(255, 255, 255);
        }
    }
}

void AtomS3LCD::printColorText(const String& input) {
    String text = input;
    uint16_t textColor = lcd.color565(255, 255, 255); // Default text color: white
    uint16_t bgColor = lcd.color565(0, 0, 0);         // Default background color: black
    int index = 0;

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
        lcd.setTextColor(textColor, bgColor);
        lcd.print(text.charAt(index));
        index++;
    }
}

void AtomS3LCD::printWaitMessage(int i2cAddress) {
    printMessage("Wait for I2C input.");
#ifdef USE_GROVE
    printColorText("\x1b[31mGROVE\x1b[39m Mode\n");
#else
    printColorText("\x1b[31mNOT GROVE\x1b[39m Mode\n");
#endif
    char log_msg[50];
    sprintf(log_msg, "I2C address \x1b[33m0x%02x\x1b[39m", i2cAddress);
    printColorText(log_msg);
}

// e.g. atoms3lcd.drawImage(atoms3lcd.jpegBuf, atoms3lcd.jpegLength);
void AtomS3LCD::drawImage(uint8_t* jpegBuf, uint32_t jpegLength) {
    lcd.drawJpg(jpegBuf, jpegLength, 0, 0, 128, 128, 0, 0, ::JPEG_DIV_NONE);
}

void AtomS3LCD::drawQRcode(const String& qrCodeData) {
    if (qrCodeData.length() > 0) {
      // Draw QR code if qrCodeData is received
      lcd.fillScreen(lcd.color565(255, 255, 255));
      lcd.qrcode(qrCodeData.c_str(), SPACING, SPACING / 2, lcd.width() - SPACING * 2, QR_VERSION);
    }
    else {
      lcd.fillScreen(lcd.color565(255, 0, 0));  // Fill the screen with red
      lcd.setCursor(0, 0);
      lcd.println("No QR code data received.");
    }
}

void AtomS3LCD::drawNoDataReceived() {
  lcd.fillScreen(lcd.color565(255, 0, 0));  // Fill the screen with red
  lcd.setCursor(0, 0);
  lcd.println("No data received.");
}

void AtomS3LCD::drawBlack() {
  lcd.fillScreen(lcd.color565(0, 0, 0));  // Fill the screen with black
  lcd.setCursor(0, 0);
}

void AtomS3LCD::resetColorStr() {
  color_str = "";
}

void AtomS3LCD::resetJpegBuf() {
  memset(jpegBuf, 0, sizeof(jpegBuf));
}

void AtomS3LCD::resetQRcodeData() {
  qrCodeData = "";
}

void AtomS3LCD::resetLcdData() {
  resetColorStr();
  resetJpegBuf();
  resetQRcodeData();
  mode_changed = true;
}

void AtomS3LCD::setTextSize(float x) {
  lcd.setTextSize(x);
}

void AtomS3LCD::fillRect(int x1, int y1, int w, int h, uint16_t color) {
  lcd.fillRect(x1, y1, w, h, color);
}

void AtomS3LCD::drawRect(int x1, int y1, int w, int h, uint16_t color) {
  lcd.drawRect(x1, y1, w, h, color);
}

void AtomS3LCD::drawLine(int x1, int y1, int x2, int y2, uint16_t color){
  lcd.drawLine(x1, y1, x2, y2, color);
};

void AtomS3LCD::drawPixel(int x, int y, uint16_t color){
  lcd.drawPixel(x, y, color);
};

void AtomS3LCD::setCursor(int x, int y){
  lcd.setCursor(x,y);
};

uint16_t AtomS3LCD::color565(uint8_t red, uint8_t green, uint8_t blue){
  return lcd.color565(red,green,blue);
};

unsigned long AtomS3LCD::getLastDrawTime() {
    return lastDrawTime;
}

void AtomS3LCD::setLastDrawTime(unsigned long time) {
  lastDrawTime = time;
}
