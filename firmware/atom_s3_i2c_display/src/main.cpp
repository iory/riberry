#include <Wire.h>
#include <WireSlave.h>
#include <OneButton.h>

#define LGFX_M5ATOMS3
#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>

static LGFX lcd;

#ifdef USE_GROVE
constexpr int SDA_PIN = 2;
constexpr int SCL_PIN = 1;
#else
constexpr int SDA_PIN = 38;
constexpr int SCL_PIN = 39;
#endif
#ifdef LCD_ROTATION
constexpr int lcd_rotation = LCD_ROTATION;
#else
constexpr int lcd_rotation = 1;
#endif

constexpr int I2C_SLAVE_ADDR = 0x42;

unsigned long lastReceiveTime = 0;
const unsigned long receiveTimeout = 15000;  // Timeout in milliseconds (15 seconds)

// for image
static const uint8_t jpegPacketHeader[3] = { 0xFF, 0xD8, 0xEA };
uint8_t jpegBuf[8000];
bool loadingJpeg = false;
uint32_t jpegLength;
uint32_t currentJpegIndex = 0;

// for QR code
static const uint8_t qrCodeHeader = 0x02;
constexpr size_t ETH_ALEN = 6;
constexpr uint16_t SPACING = 4;
constexpr uint8_t FONT = 1;
constexpr uint8_t FONT_HEIGHT = 8;
constexpr uint8_t QR_VERSION = 1;

void receiveEvent(int howMany);
void requestEvent();

// ==== Button ==== //
constexpr int BUTTON_PIN = 41;
OneButton btn(BUTTON_PIN, true, false);
enum ButtonState {
  NOT_CHANGED,
  SINGLE_CLICK,
  DOUBLE_CLICK,
  TRIPLE_CLICK,
  QUADRUPLE_CLICK,  // 4 clicks
  QUINTUPLE_CLICK,  // 5 clicks
  SEXTUPLE_CLICK,   // 6 clicks
  SEPTUPLE_CLICK,   // 7 clicks
  OCTUPLE_CLICK,    // 8 clicks
  NONUPLE_CLICK,    // 9 clicks
  DECUPLE_CLICK,    // 10 clicks
  PRESSED,
  RELEASED,
  BUTTON_STATE_COUNT
};
ButtonState currentButtonState = NOT_CHANGED;

static void handleClick() {
  currentButtonState = SINGLE_CLICK;
}

static void handleDoubleClick() {
  currentButtonState = DOUBLE_CLICK;
}

static void handleMultiClick() {
  int n = btn.getNumberClicks();
  switch (n) {
  case 1: currentButtonState = SINGLE_CLICK; break;
  case 2: currentButtonState = DOUBLE_CLICK; break;
  case 3: currentButtonState = TRIPLE_CLICK; break;
  case 4: currentButtonState = QUADRUPLE_CLICK; break;
  case 5: currentButtonState = QUINTUPLE_CLICK; break;
  case 6: currentButtonState = SEXTUPLE_CLICK; break;
  case 7: currentButtonState = SEPTUPLE_CLICK; break;
  case 8: currentButtonState = OCTUPLE_CLICK; break;
  case 9: currentButtonState = NONUPLE_CLICK; break;
  case 10: currentButtonState = DECUPLE_CLICK; break;
  default: currentButtonState = DECUPLE_CLICK; // Handle case where n > 10
  }
}

static void handleLongPress() {
  currentButtonState = PRESSED;
}

static void handleLongPressEnd() {
  currentButtonState = RELEASED;
}

void ButtonTask(void *parameter) {
  btn.setClickMs(200);  // Timeout used to distinguish single clicks from double clicks. (msec)
  btn.attachClick(handleClick);
  btn.attachDoubleClick(handleDoubleClick);
  btn.attachMultiClick(handleMultiClick);
  btn.attachLongPressStart(handleLongPress);
  btn.attachLongPressStop(handleLongPressEnd);// Initialize last receive time

  while (true) {
    btn.tick();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}


void I2CTask(void *parameter) {
  bool success = WireSlave.begin(SDA_PIN, SCL_PIN, I2C_SLAVE_ADDR);
  if (!success) {
    lcd.println("I2C slave init failed");
    while (1) delay(100);
  }
  WireSlave.onReceive(receiveEvent);
  WireSlave.onRequest(requestEvent);
  while (true) {
    WireSlave.update();
    delay(1);  // let I2C and other ESP32 peripherals interrupts work
  }
}

uint16_t colorMap(int code, bool isBackground = false) {
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


void printColorText(const String& input) {
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


void setup() {
    // For display
    lcd.init();
    lcd.setRotation(lcd_rotation);
    lcd.clear();
    lcd.setTextSize(1.5);

    lcd.println("Wait for I2C input.");
#ifdef USE_GROVE
    printColorText("\x1b[31mGROVE\x1b[39m Mode\n");
#else
    printColorText("\x1b[31mNOT GROVE\x1b[39m Mode\n");
#endif
    char log_msg[50];
    sprintf(log_msg, "I2C address \x1b[33m0x%02x\x1b[39m", I2C_SLAVE_ADDR);
    printColorText(log_msg);

    lastReceiveTime = millis();

    xTaskCreatePinnedToCore(I2CTask, "I2C Task", 2048, NULL, 24, NULL, 0);
    xTaskCreatePinnedToCore(ButtonTask, "Button Task", 2048, NULL, 24, NULL, 1);
}

void loop() {
    if (millis() - lastReceiveTime > receiveTimeout) {
        lcd.fillScreen(lcd.color565(255, 0, 0));  // Fill the screen with red
        lcd.setCursor(0, 0);
        lcd.println("No data received.");
        delay(500);  // Show message for a short time
    }
}

void receiveEvent(int howMany) {
    lastReceiveTime = millis();  // Update the last received time
    String str;
    while (0 < WireSlave.available()) {
        char c = WireSlave.read();  // receive byte as a character
        if (loadingJpeg && str.length() >= 3) {
          jpegBuf[currentJpegIndex + str.length() - 3] = c;
        }
        str += c;
    }
    if (str.length() == 5
        && (str[0] == jpegPacketHeader[0]) && (str[1] == jpegPacketHeader[1]) && (str[2] == jpegPacketHeader[2])) {
      jpegLength = (uint32_t)(str[3] << 8) | str[4];
      currentJpegIndex = 0;
      loadingJpeg = true;
      return;
    } else if (loadingJpeg) {
      if ((str[0] == jpegPacketHeader[0]) && (str[1] == jpegPacketHeader[1]) && (str[2] == jpegPacketHeader[2])) {
        currentJpegIndex += str.length() - 3;
        if (currentJpegIndex >= jpegLength) {
          lcd.drawJpg(jpegBuf, jpegLength, 0, 0, 128, 128, 0, 0, ::JPEG_DIV_NONE);
          loadingJpeg = false;
        }
        return;
      } else {
        loadingJpeg = false;
      }
    }
    if (str.length() > 1 && str[0] == qrCodeHeader) {
      uint8_t qrCodeLength = str[1];  // Assuming the length of the QR code data is in the second byte
      String qrCodeData = str.substring(2, 2 + qrCodeLength);
      lcd.fillScreen(lcd.color565(255, 255, 255));
      lcd.qrcode(qrCodeData.c_str(), SPACING, SPACING / 2, lcd.width() - SPACING * 2, QR_VERSION);
      return;
    }

    // Draw
    lcd.fillScreen(lcd.color565(0, 0, 0));
    lcd.setCursor(0, 0);
    printColorText(str);
}

void requestEvent() {
  WireSlave.write(currentButtonState);
  currentButtonState = NOT_CHANGED;
}
