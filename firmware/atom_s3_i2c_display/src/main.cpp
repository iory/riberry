#include <M5AtomS3.h>
#include <Wire.h>
#include <WireSlave.h>
#include <WireUnpacker.h>

WireUnpacker unpacker(256);

constexpr int SDA_PIN = 38;
constexpr int SCL_PIN = 39;
constexpr int I2C_SLAVE_ADDR = 0x42;

Stream* target_serial;
String target_serial_type = "i2c";  // For testing purposes, set to "i2c"
// String target_serial_type = "serial1";

void receiveEvent(int howMany);

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
        sprintf(log_msg, "I2C address 0x%02x", I2C_SLAVE_ADDR);
        M5.Lcd.println(log_msg);

        bool success = WireSlave.begin(SDA_PIN, SCL_PIN, I2C_SLAVE_ADDR);
        if (!success) {
            M5.Lcd.println("I2C slave init failed");
            while (1) delay(100);
        }

        WireSlave.onReceive(receiveEvent);
    } else {
        M5.Lcd.fillScreen(M5.Lcd.color565(0, 0, 0));
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.println("invalid target_serial_type.");
        while (true) delay(100);
    }
}

void loop() {
    if (target_serial_type == "i2c") {
        WireSlave.update();
        delay(1);  // let I2C and other ESP32 peripherals interrupts work
    } else {
        String str;
        if (target_serial->available() > 0) {
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
    // Clear display
    M5.Lcd.fillScreen(M5.Lcd.color565(0, 0, 0));
    M5.Lcd.setCursor(0, 0);
    String str;
    while (0 < WireSlave.available()) {
        char c = WireSlave.read();  // receive byte as a character
        str += c;
    }
    // Draw
    M5.Lcd.println(str);
}
