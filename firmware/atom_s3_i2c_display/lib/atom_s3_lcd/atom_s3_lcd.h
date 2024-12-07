#ifndef ATOM_S3_LCD_H
#define ATOM_S3_LCD_H

#define LGFX_M5ATOMS3
#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>

#define LCD_W 128
#define LCD_H 128

/**
 * @brief Class to handle the LCD on the AtomS3 using the LovyanGFX library.
 */
class AtomS3LCD {
public:
  /**
   * @brief Constructor to initialize the LCD with a specific rotation.
   *
   * @param rotation The screen rotation (e.g., landscape, portrait).
   */
  AtomS3LCD();

  /**
   * @brief Initialize the LCD screen and set default properties.
   */
  void init();

  /**
   * @brief Clear the LCD screen.
   */
  void clear();

  /**
   * @brief Display a wait message and show whether GROVE mode is active.
   *
   * @param i2cAddress The I2C address to display on the screen.
   */
  void printWaitMessage(int i2cAddress);

  /**
   * @brief Print a message to the LCD screen.
   *
   * @param message The message to be printed.
   */
  void printMessage(const String& message);

  /**
   * @brief Print colored text using ANSI-style escape sequences.
   *
   * @param input The input string containing ANSI escape codes.
   */
  void printColorText(const String& input);

  /**
   * @brief Draw a JPEG image on the LCD screen.
   *
   * @param jpegBuf A buffer containing the JPEG image data.
   * @param jpegLength The length of the JPEG data.
   */
  void drawImage(uint8_t* jpegBuf, uint32_t jpegLength);

  /**
   * @brief Draw a QR code on the LCD screen.
   *
   * @param qrCodeData The data to encode in the QR code.
   */
  void drawQRcode(const String& qrCodeData);

  /**
   * @brief Display a "No data received" error message on the screen.
   */
  void drawNoDataReceived();

  /**
   * @brief Fill the screen with black color.
   */
  void drawBlack();

  void resetColorStr();
  void resetJpegBuf();
  void resetQRcodeData();
  void resetLcdData();
  void setTextSize(float x);
  void fillRect(int x1, int y1, int x2, int y2, uint16_t color);
  void drawRect(int x1, int y1, int x2, int y2, uint16_t color);
  void drawLine(int x1, int y1, int x2, int y2, uint16_t color);
  void drawPixel(int x, int y, uint16_t color);
  void setCursor(int x, int y);
  uint16_t color565(uint8_t red, uint8_t green, uint8_t blue);
  unsigned long getLastDrawTime();
  void setLastDrawTime(unsigned long time);

  bool mode_changed = false;

  // for color string
  String color_str; /**< Stores color-related text for display. */

  // for image
  uint8_t jpegBuf[8000]; /**< Buffer to store JPEG image data. */
  bool loadingJpeg = false; /**< Indicates whether the system is currently loading a JPEG image. */
  bool readyJpeg = false; /**< Indicates whether the JPEG image has been fully loaded and is ready for use. */
  uint32_t jpegLength; /**< Length of the JPEG image data in bytes. */
  uint32_t currentJpegIndex = 0; /**< Current index in the JPEG buffer. */

  // for QR code
  static constexpr size_t ETH_ALEN = 6; /**< Size of an Ethernet address. */
  static constexpr uint16_t SPACING = 4; /**< Spacing used when rendering the QR code. */
  static constexpr uint8_t FONT = 1; /**< Default font used for displaying text. */
  static constexpr uint8_t FONT_HEIGHT = 8; /**< Height of the default font in pixels. */
  static constexpr uint8_t QR_VERSION = 1; /**< QR code version used for encoding. */
  String qrCodeData; /**< Stores the data to be encoded in the QR code. */

private:
  LGFX lcd;  /**< Instance of LovyanGFX for controlling the LCD. */
#ifdef LCD_ROTATION
  static constexpr int lcd_rotation = LCD_ROTATION; /**< Current rotation of the LCD. */
#else
  static constexpr int lcd_rotation = 1; /**< Current rotation of the LCD. */
#endif
  unsigned long lastDrawTime = 0;
  /**
   * @brief Convert ANSI color codes to RGB565 values for foreground or background.
   *
   * @param code The ANSI color code.
   * @param isBackground Whether the color is for the background (true) or foreground (false).
   * @return The RGB565 color value.
   */
  uint16_t colorMap(int code, bool isBackground = false);
};

#endif // ATOM_S3_LCD_H
