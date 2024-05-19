#include <M5AtomS3.h>
#include <Wire.h>
#include <WireSlave.h>
#include <WireUnpacker.h>
#include <OneButton.h>

WireUnpacker unpacker(256);

#ifdef USE_GROVE
constexpr int SDA_PIN = 2;
constexpr int SCL_PIN = 1;
#else
constexpr int SDA_PIN = 38;
constexpr int SCL_PIN = 39;
#endif
constexpr int I2C_SLAVE_ADDR = 0x42;

unsigned long lastReceiveTime = 0;
const unsigned long receiveTimeout = 15000;  // Timeout in milliseconds (15 seconds)

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
    M5.Lcd.println("I2C slave init failed");
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
        case 40: return M5.Lcd.color565(0, 0, 0);     // Black
        case 41: return M5.Lcd.color565(255, 0, 0);   // Red
        case 42: return M5.Lcd.color565(0, 255, 0);   // Green
        case 43: return M5.Lcd.color565(255, 255, 0); // Yellow
        case 44: return M5.Lcd.color565(0, 0, 255);   // Blue
        case 45: return M5.Lcd.color565(255, 0, 255); // Magenta
        case 46: return M5.Lcd.color565(0, 255, 255); // Cyan
        case 47: return M5.Lcd.color565(255, 255, 255); // White
        case 49: return M5.Lcd.color565(0, 0, 0);     // Default (Black)
        default: return M5.Lcd.color565(0, 0, 0);
        }
    } else {
        switch (code) {
        case 30: return M5.Lcd.color565(0, 0, 0);     // Black
        case 31: return M5.Lcd.color565(255, 0, 0);   // Red
        case 32: return M5.Lcd.color565(0, 255, 0);   // Green
        case 33: return M5.Lcd.color565(255, 255, 0); // Yellow
        case 34: return M5.Lcd.color565(0, 0, 255);   // Blue
        case 35: return M5.Lcd.color565(255, 0, 255); // Magenta
        case 36: return M5.Lcd.color565(0, 255, 255); // Cyan
        case 37: return M5.Lcd.color565(255, 255, 255); // White
        case 39: return M5.Lcd.color565(255, 255, 255); // Default (White)
        default: return M5.Lcd.color565(255, 255, 255);
        }
    }
}


void printColorText(const String& input) {
    String text = input;
    uint16_t textColor = M5.Lcd.color565(255, 255, 255); // Default text color: white
    uint16_t bgColor = M5.Lcd.color565(0, 0, 0);         // Default background color: black
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
        M5.Lcd.setTextColor(textColor, bgColor);
        M5.Lcd.print(text.charAt(index));
        index++;
    }
}


void setup() {
    M5.begin();

    // For display
    M5.Lcd.setRotation(1);
    M5.Lcd.setTextSize(1.5);

    M5.Lcd.println("Wait for I2C input.");
    char log_msg[50];
    sprintf(log_msg, "I2C address \x1b[33m0x%02x\x1b[39m", I2C_SLAVE_ADDR);
    printColorText(log_msg);

    lastReceiveTime = millis();

    xTaskCreatePinnedToCore(I2CTask, "I2C Task", 2048, NULL, 24, NULL, 0);
    xTaskCreatePinnedToCore(ButtonTask, "Button Task", 2048, NULL, 24, NULL, 1);
}

void loop() {
    if (millis() - lastReceiveTime > receiveTimeout) {
        M5.Lcd.fillScreen(M5.Lcd.color565(255, 0, 0));  // Fill the screen with red
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.println("No data received.");
        delay(500);  // Show message for a short time
    }
}

void receiveEvent(int howMany) {
    lastReceiveTime = millis();  // Update the last received time

    // Clear display
    M5.Lcd.fillScreen(M5.Lcd.color565(0, 0, 0));
    M5.Lcd.setCursor(0, 0);
    String str;
    while (0 < WireSlave.available()) {
        char c = WireSlave.read();  // receive byte as a character
        str += c;
    }
    // Draw
    printColorText(str);
}

void requestEvent() {
  WireSlave.write(currentButtonState);
  currentButtonState = NOT_CHANGED;
}
