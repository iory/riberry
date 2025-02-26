#ifndef PRIMITIVE_LCD_H
#define PRIMITIVE_LCD_H

#ifdef ATOM_S3
    #define LGFX_M5ATOMS3
constexpr float DEFAULT_TEXT_SIZE = 1.5;
#elif defined(USE_M5STACK_BASIC)
    #define LGFX_M5STACK
constexpr float DEFAULT_TEXT_SIZE = 3.0;
#endif
#define LGFX_USE_V1
#include <LGFX_AUTODETECT.hpp>
#include <LovyanGFX.hpp>

class PrimitiveLCD : public LGFX {
public:
    PrimitiveLCD();
    void drawJpg(const uint8_t* jpg_data,
                 size_t jpg_len,
                 uint16_t x = 0,
                 uint16_t y = 0,
                 uint16_t maxWidth = 0,
                 uint16_t maxHeight = 0,
                 uint16_t offX = 0,
                 uint16_t offY = 0,
                 jpeg_div_t scale = JPEG_DIV_NONE);
    void qrcode(const char* string, uint16_t x, uint16_t y, uint8_t width, uint8_t version);
    void setJapaneseFont();
    void printColorText(const String& input);
    void fillScreen(uint16_t color);
    void fillRect(int x1, int y1, int w, int h, uint16_t color);
    void fillCircle(int x, int y, int r, uint16_t color);
    void drawRect(int x1, int y1, int w, int h, uint16_t color);
    void drawCircle(int x, int y, int r, uint16_t color);
    void drawNumber(long long_num, int32_t posX, int32_t posY);
    void drawLine(int x1, int y1, int x2, int y2, uint16_t color);
    void drawPixel(int x, int y, uint16_t color);
    void setCursor(int x, int y);
    void setRotation(int lcd_rotation);
    void clear();
    void setTextSize(float text_size);
    float getTextSize();
    void setTextDatum(textdatum_t datum);
    uint16_t colorMap(int code, bool isBackground);
    void drawImage(uint8_t* jpegBuf, uint32_t jpegLength);
    void drawQRcode(const String& qrCodeData);
    void printWaitMessage(int i2cAddress);
    void drawNoDataReceived();
    void drawBlack();

    void resetColorStr();
    void resetJpegBuf();
    void resetQRcodeData();
    void resetLcdData();
    unsigned long getLastDrawTime();
    void setLastDrawTime(unsigned long time);

    bool lockLcd();
    void unlockLcd();

    bool mode_changed = false;

    // for color string
    String color_str; /**< Stores color-related text for display. */

    // for image
    uint8_t jpegBuf[8000];         /**< Buffer to store JPEG image data. */
    bool loadingJpeg = false;      /**< Indicates whether the system is currently
                                      loading a JPEG image. */
    bool readyJpeg = false;        /**< Indicates whether the JPEG image has been fully
                                      loaded and is ready for use. */
    uint32_t jpegLength;           /**< Length of the JPEG image data in bytes. */
    uint32_t currentJpegIndex = 0; /**< Current index in the JPEG buffer. */

    // for QR code
    static constexpr size_t ETH_ALEN = 6;     /**< Size of an Ethernet address. */
    static constexpr uint16_t SPACING = 4;    /**< Spacing used when rendering the QR code. */
    static constexpr uint8_t FONT = 1;        /**< Default font used for displaying text. */
    static constexpr uint8_t FONT_HEIGHT = 8; /**< Height of the default font in pixels. */
    static constexpr uint8_t QR_VERSION = 1;  /**< QR code version used for encoding. */
    String qrCodeData;                        /**< Stores the data to be encoded in the QR code. */

private:
    SemaphoreHandle_t lcdMutex;
    float textSize = 0;

#ifdef LCD_ROTATION
    static constexpr int lcd_rotation = LCD_ROTATION; /**< Current rotation of the LCD. */
#else
    static constexpr int lcd_rotation = 1; /**< Current rotation of the LCD. */
#endif
    unsigned long lastDrawTime = 0;
};

#endif  // PRIMITIVE_LCD_H
