#include <atom_s3_i2c.h>

AtomS3I2C* AtomS3I2C::instance = nullptr;

AtomS3I2C::AtomS3I2C(AtomS3LCD &lcd, AtomS3Button &button)
  : atoms3lcd(lcd), atoms3button(button){
  instance = this;
  createTask(0);
}

void AtomS3I2C::updateLastReceiveTime() {
  lastReceiveTime = millis();
}

bool AtomS3I2C::checkTimeout() {
  return millis() - lastReceiveTime > receiveTimeout;
}

// Receive data over I2C and store it in the atoms3lcd instance (color_str, jpegBuf, qrCodeData)
// No rendering is done; lcd update is handled by other tasks
void AtomS3I2C::receiveEvent(int howMany) {
    if (instance == nullptr)
      return;
    instance->updateLastReceiveTime();
    String str;
    // Read data from the I2C bus
    while (0 < WireSlave.available()) {
        char c = WireSlave.read();  // receive byte as a character
        // If currently loading JPEG, store the data in jpegBuf
        if (instance->atoms3lcd.loadingJpeg && str.length() >= 3) {
            instance->atoms3lcd.jpegBuf[instance->atoms3lcd.currentJpegIndex + str.length() - 3] = c;
        }
        str += c;
    }
    // Check for JPEG packet header and update length if found
    if (str.length() == 5 && (str[0] == instance->atoms3lcd.jpegPacketHeader[0]) && (str[1] == instance->atoms3lcd.jpegPacketHeader[1]) && (str[2] == instance->atoms3lcd.jpegPacketHeader[2])) {
        instance->atoms3lcd.jpegLength = (uint32_t)(str[3] << 8) | str[4];
        instance->atoms3lcd.currentJpegIndex = 0;
        instance->atoms3lcd.loadingJpeg = true;
        return;
    }
    // Continue receiving JPEG data if already in loading state
    else if (instance->atoms3lcd.loadingJpeg) {
        if ((str[0] == instance->atoms3lcd.jpegPacketHeader[0]) && (str[1] == instance->atoms3lcd.jpegPacketHeader[1]) && (str[2] == instance->atoms3lcd.jpegPacketHeader[2])) {
            instance->atoms3lcd.currentJpegIndex += str.length() - 3;
            // End loading if the entire JPEG is received
            if (instance->atoms3lcd.currentJpegIndex >= instance->atoms3lcd.jpegLength) {
                instance->atoms3lcd.loadingJpeg = false;
            }
            return;
        } else {
          // Stop loading if a wrong packet is received
          instance->atoms3lcd.loadingJpeg = false;
        }
    }
    // Check for QR code header and store data if found
    if (str.length() > 1 && str[0] == instance->atoms3lcd.qrCodeHeader) {
        uint8_t qrCodeLength = str[1];  // Assuming the length of the QR code data is in the second byte
        instance->atoms3lcd.qrCodeData = str.substring(2, 2 + qrCodeLength);
        return;
    }
    instance->atoms3lcd.color_str = str;
}

void AtomS3I2C::requestEvent() {
  if (instance == nullptr)
      return;
  WireSlave.write(instance->atoms3button.getButtonState());
  instance->atoms3button.notChangedButtonState();
}

void AtomS3I2C::task(void *parameter) {
  bool success = WireSlave.begin(sda_pin, scl_pin, i2c_slave_addr, 200, 100);
  if (!success) {
    instance->atoms3lcd.printMessage("I2C slave init failed");
    while (1) vTaskDelay(pdMS_TO_TICKS(100));;
  }
  // Ensure the program starts in a timeout state
  instance->lastReceiveTime = millis() - instance->receiveTimeout;
  WireSlave.onReceive(receiveEvent);
  WireSlave.onRequest(requestEvent);
  while (true) {
    WireSlave.update();    
    vTaskDelay(pdMS_TO_TICKS(1));;  // let I2C and other ESP32 peripherals interrupts work
  }
}

void AtomS3I2C::createTask(uint8_t xCoreID) {
  xTaskCreatePinnedToCore(task, "I2C Task", 2048, this, 24, NULL, xCoreID);
}
