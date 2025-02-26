#include <primitive_lcd.h>

PrimitiveLCD::PrimitiveLCD() : LGFX(), qrCodeData("") {
    init();
    lcdMutex = xSemaphoreCreateRecursiveMutex();
    setRotation(lcd_rotation);
    clear();
    setTextSize(DEFAULT_TEXT_SIZE);
}

void PrimitiveLCD::drawJpg(const uint8_t* jpg_data,
                           size_t jpg_len,
                           uint16_t x,
                           uint16_t y,
                           uint16_t maxWidth,
                           uint16_t maxHeight,
                           uint16_t offX,
                           uint16_t offY,
                           jpeg_div_t scale) {
    if (lockLcd()) {
        LGFX::drawJpg(jpg_data, jpg_len, x, y, maxWidth, maxHeight, offX, offY, scale);
        unlockLcd();
    }
}

void PrimitiveLCD::qrcode(
        const char* string, uint16_t x, uint16_t y, uint8_t width, uint8_t version) {
    if (lockLcd()) {
        LGFX::qrcode(string, x, y, width, version);
        unlockLcd();
    }
}

void PrimitiveLCD::setJapaneseFont() {
#ifdef ATOM_S3
    setFont(&fonts::lgfxJapanGothic_8);
#elif USE_M5STACK_BASIC
    setFont(&fonts::lgfxJapanGothic_12);
#endif
}

void PrimitiveLCD::printColorText(const String& input) {
    String text = input;
    uint16_t textColor = LGFX::color565(255, 255, 255);  // Default text color: white
    uint16_t bgColor = LGFX::color565(0, 0, 0);          // Default background color: black
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
            uint8_t firstByte = text.charAt(index);
            int charLen;
            if ((firstByte & 0x80) == 0) {  // 0xxxxxxx - ASCII
                charLen = 1;
            } else if ((firstByte & 0xE0) == 0xC0) {  // 110xxxxx - 2 byte character
                charLen = 2;
                setJapaneseFont();
            } else if ((firstByte & 0xF0) == 0xE0) {  // 1110xxxx - 3 byte character
                charLen = 3;
                setJapaneseFont();
            } else if ((firstByte & 0xF8) == 0xF0) {  // 11110xxx - 4 byte character
                charLen = 4;
                setJapaneseFont();
            } else {  // Invalid UTF-8 byte
                charLen = 1;
            }
            LGFX::print(text.substring(index, index + charLen));
            setFont(&fonts::Font0);
            index += charLen;
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

void PrimitiveLCD::fillCircle(int x, int y, int r, uint16_t color) {
    if (lockLcd()) {
        LGFX::fillCircle(x, y, r, color);
        unlockLcd();
    }
}

void PrimitiveLCD::drawRect(int x1, int y1, int w, int h, uint16_t color) {
    if (lockLcd()) {
        LGFX::drawRect(x1, y1, w, h, color);
        unlockLcd();
    }
}

void PrimitiveLCD::drawCircle(int x, int y, int r, uint16_t color) {
    if (lockLcd()) {
        LGFX::drawCircle(x, y, r, color);
        unlockLcd();
    }
}

void PrimitiveLCD::drawNumber(long long_num, int32_t posX, int32_t posY) {
    if (lockLcd()) {
        LGFX::drawNumber(long_num, posX, posY);
        unlockLcd();
    }
};

void PrimitiveLCD::drawLine(int x1, int y1, int x2, int y2, uint16_t color) {
    if (lockLcd()) {
        LGFX::drawLine(x1, y1, x2, y2, color);
        unlockLcd();
    }
};

void PrimitiveLCD::drawPixel(int x, int y, uint16_t color) {
    if (lockLcd()) {
        LGFX::drawPixel(x, y, color);
        unlockLcd();
    }
};

void PrimitiveLCD::setCursor(int x, int y) {
    if (lockLcd()) {
        LGFX::setCursor(x, y);
        unlockLcd();
    }
};

void PrimitiveLCD::setRotation(int lcd_rotation) {
    if (lockLcd()) {
        LGFX::setRotation(lcd_rotation);
        unlockLcd();
    }
}

void PrimitiveLCD::clear() {
    if (lockLcd()) {
        LGFX::clear();
        unlockLcd();
    }
}

void PrimitiveLCD::setTextSize(float text_size) {
    textSize = text_size;
    if (lockLcd()) {
        LGFX::setTextSize(text_size);
        unlockLcd();
    }
}

float PrimitiveLCD::getTextSize() { return textSize; }

void PrimitiveLCD::setTextDatum(textdatum_t datum) {
    if (lockLcd()) {
        LGFX::setTextDatum(datum);
        unlockLcd();
    }
}

uint16_t PrimitiveLCD::colorMap(int code, bool isBackground) {
    if (isBackground) {
        switch (code) {
            case 40:
                return LGFX::color565(0, 0, 0);  // Black
            case 41:
                return LGFX::color565(255, 0, 0);  // Red
            case 42:
                return LGFX::color565(0, 255, 0);  // Green
            case 43:
                return LGFX::color565(255, 255, 0);  // Yellow
            case 44:
                return LGFX::color565(0, 0, 255);  // Blue
            case 45:
                return LGFX::color565(255, 0, 255);  // Magenta
            case 46:
                return LGFX::color565(0, 255, 255);  // Cyan
            case 47:
                return LGFX::color565(255, 255, 255);  // White
            case 49:
                return LGFX::color565(0, 0, 0);  // Default (Black)
            default:
                return LGFX::color565(0, 0, 0);
        }
    } else {
        switch (code) {
            case 30:
                return LGFX::color565(0, 0, 0);  // Black
            case 31:
                return LGFX::color565(255, 0, 0);  // Red
            case 32:
                return LGFX::color565(0, 255, 0);  // Green
            case 33:
                return LGFX::color565(255, 255, 0);  // Yellow
            case 34:
                return LGFX::color565(0, 0, 255);  // Blue
            case 35:
                return LGFX::color565(255, 0, 255);  // Magenta
            case 36:
                return LGFX::color565(0, 255, 255);  // Cyan
            case 37:
                return LGFX::color565(255, 255, 255);  // White
            case 39:
                return LGFX::color565(255, 255, 255);  // Default (White)
            default:
                return LGFX::color565(255, 255, 255);
        }
    }
}

bool PrimitiveLCD::lockLcd() { return xSemaphoreTakeRecursive(lcdMutex, portMAX_DELAY) == pdTRUE; }

void PrimitiveLCD::unlockLcd() { xSemaphoreGiveRecursive(lcdMutex); }

// e.g. lcd.drawImage(lcd.jpegBuf, lcd.jpegLength);
void PrimitiveLCD::drawImage(uint8_t* jpegBuf, uint32_t jpegLength) {
    drawJpg(jpegBuf, jpegLength, 0, 0, width(), height(), 0, 0, ::JPEG_DIV_NONE);
}

void PrimitiveLCD::drawQRcode(const String& qrCodeData) {
    if (qrCodeData.length() > 0) {
        // Draw QR code if qrCodeData is received
        fillScreen(color565(255, 255, 255));
        int rectWidth = min(width(), height());
        qrcode(qrCodeData.c_str(), static_cast<int>(width() / 2 - rectWidth / 2) + SPACING,
               static_cast<int>(height() / 2 - rectWidth / 2) + SPACING / 2,
               rectWidth - SPACING * 2, QR_VERSION);
    } else {
        fillScreen(color565(255, 0, 0));  // Fill the screen with red
        setCursor(0, 0);
        println("No QR code data received.");
    }
}

void PrimitiveLCD::printWaitMessage(int i2cAddress) {
#ifdef ATOM_S3
    printColorText("Wait for I2C input.\n");
    #ifdef USE_GROVE
    printColorText("\x1b[31mGROVE\x1b[39m Mode\n");
    #else
    printColorText("\x1b[31mNOT GROVE\x1b[39m Mode\n");
    #endif  // end of USE_GROVE

#elif defined(USE_M5STACK_BASIC)
    printColorText("Wait for /dev/ttyS1 input.\n");
#endif
    char log_msg[50];
    sprintf(log_msg, "I2C address \x1b[33m0x%02x\x1b[39m", i2cAddress);
    printColorText(log_msg);
}

void PrimitiveLCD::drawNoDataReceived() {
    fillScreen(color565(255, 0, 0));  // Fill the screen with red
    setCursor(0, 0);
    printColorText("No data received.\n");
}

void PrimitiveLCD::drawBlack() {
    fillScreen(color565(0, 0, 0));  // Fill the screen with black
    setCursor(0, 0);
}

void PrimitiveLCD::resetColorStr() { color_str = ""; }

void PrimitiveLCD::resetJpegBuf() { memset(jpegBuf, 0, sizeof(jpegBuf)); }

void PrimitiveLCD::resetQRcodeData() { qrCodeData = ""; }

void PrimitiveLCD::resetLcdData() {
    resetColorStr();
    resetJpegBuf();
    resetQRcodeData();
    mode_changed = true;
}

unsigned long PrimitiveLCD::getLastDrawTime() { return lastDrawTime; }

void PrimitiveLCD::setLastDrawTime(unsigned long time) { lastDrawTime = time; }
