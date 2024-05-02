#include <M5AtomS3.h>
#include <Wire.h>
#include <WireSlave.h>
#include <WireUnpacker.h>
#include <OneButton.h>

WireUnpacker unpacker(256);

constexpr int SDA_PIN = 38;
constexpr int SCL_PIN = 39;
constexpr int I2C_SLAVE_ADDR = 0x42;

unsigned long lastReceiveTime = 0;
const unsigned long receiveTimeout = 15000;  // Timeout in milliseconds (15 seconds)

Stream* target_serial;
String target_serial_type = "i2c";  // For testing purposes, set to "i2c"
// String target_serial_type = "serial1";

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

void ButtonTask(void *parameter) {
  while (true) {
    btn.tick();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

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

    // Determine the target serial type
    if (target_serial_type == "serial1") {
        M5.Lcd.println("Wait for UART input.");
        M5.Lcd.println("Baud rate = 1000000,");
        M5.Lcd.println("SERIAL_8N1, 1, 2.");
        delay(500);
        Serial1.begin(1000000, SERIAL_8N1, 1, 2);
        target_serial = &Serial1;
    } else if (target_serial_type == "i2c") {
        M5.Lcd.println("Wait for I2C input.");
        char log_msg[50];
        sprintf(log_msg, "I2C address \x1b[33m0x%02x\x1b[39m", I2C_SLAVE_ADDR);
        printColorText(log_msg);

        bool success = WireSlave.begin(SDA_PIN, SCL_PIN, I2C_SLAVE_ADDR);
        if (!success) {
            M5.Lcd.println("I2C slave init failed");
            while (1) delay(100);
        }
        WireSlave.onReceive(receiveEvent);
        WireSlave.onRequest(requestEvent);
    } else {
        M5.Lcd.fillScreen(M5.Lcd.color565(0, 0, 0));
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.println("invalid target_serial_type.");
        while (true) delay(100);
    }
    lastReceiveTime = millis();

    btn.setClickMs(200);  // Timeout used to distinguish single clicks from double clicks. (msec)
    btn.attachClick(handleClick);
    btn.attachDoubleClick(handleDoubleClick);
    btn.attachMultiClick(handleMultiClick);
    btn.attachLongPressStart(handleLongPress);
    btn.attachLongPressStop(handleLongPressEnd);// Initialize last receive time

    xTaskCreatePinnedToCore(ButtonTask, "Button Task", 2048, NULL, 25, NULL, 0);
}

void loop() {
    if (millis() - lastReceiveTime > receiveTimeout) {
        M5.Lcd.fillScreen(M5.Lcd.color565(255, 0, 0));  // Fill the screen with red
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.println("No data received.");
        delay(500);  // Show message for a short time
    }
    if (target_serial_type == "i2c") {
        WireSlave.update();
        delay(1);  // let I2C and other ESP32 peripherals interrupts work
    } else {
        String str;
        if (target_serial->available() > 0) {
            lastReceiveTime = millis();  // Update the last received time
            // Clear display
            M5.Lcd.fillScreen(M5.Lcd.color565(0, 0, 0));
            M5.Lcd.setCursor(0, 0);
            unpacker.reset();
            int incoming_byte = target_serial->read();
            while (incoming_byte != -1 || !unpacker.available()) {
                unpacker.write(incoming_byte);
                incoming_byte = target_serial->read();
            }
            while (unpacker.available()) {
                str += (char)unpacker.read();
            }
            // Draw
            M5.Lcd.println(str);
        }
        delay(1000);
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
