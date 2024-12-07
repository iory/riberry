#include <atom_s3_i2c.h>

AtomS3I2C* AtomS3I2C::instance = nullptr;
String AtomS3I2C::requestStr = ""; // Initialize the static requestStr
String AtomS3I2C::forcedMode = ""; // Initialize the static forcedMode
String AtomS3I2C::selectedModesStr = ""; // Initialize the static selectedModesStr

AtomS3I2C::AtomS3I2C(AtomS3LCD &lcd, AtomS3Button &button)
  : atoms3lcd(lcd), atoms3button(button), receiveEventEnabled(true) {
  instance = this;
}

void AtomS3I2C::updateLastReceiveTime() {
  lastReceiveTime = millis();
}

bool AtomS3I2C::checkTimeout() {
  return millis() - lastReceiveTime > receiveTimeout;
}

// Use char* instead of String for output[] argument
// to minimize dynamic memory usage and reduce the risk of fragmentation.
int AtomS3I2C::splitString(const String &input, char delimiter, char* output[], int maxParts) {
  int start = 0;
  int index = 0;

  while (true) {
    int end = input.indexOf(delimiter, start);
    if (end == -1) { // 区切り文字が見つからない場合
      if (index < maxParts) {
        String part = input.substring(start);       // 最後の部分を取得
        output[index] = strdup(part.c_str());       // strdupで動的メモリにコピー
        index++;
      }
      break;
    }
    if (index < maxParts) {
      String part = input.substring(start, end);   // 区切り文字までの部分を取得
      output[index] = strdup(part.c_str());        // strdupで動的メモリにコピー
      index++;
    }
    start = end + 1; // 次の部分へ進む
  }

  return index; // 分割された部分の数を返す
}


// Receive data over I2C and store it in the atoms3lcd instance
// No rendering is done; lcd update is handled by other tasks
void AtomS3I2C::receiveEvent(int howMany) {
    if (instance->receiveEventEnabled == false)
      return;
    if (instance == nullptr)
      return;
    instance->updateLastReceiveTime();
    String str;
    // Read data from the I2C bus
    while (0 < WireSlave.available()) {
        char c = WireSlave.read();  // receive byte as a character;
        str += c;
    }

    // Parse the first byte as PacketType
    if (str.length() < 1) {
        return;  // Invalid packet
    }

    PacketType packetType = static_cast<PacketType>(str[0]);
    switch (packetType) {
        case JPEG:
            handleJpegPacket(str.substring(1));
            break;

        case QR_CODE:
            handleQrCodePacket(str);
            break;

        case FORCE_MODE:
            handleForceModePacket(str);
            break;

        case SELECTED_MODE:
            handleSelectedModePacket(str);
            break;

        case DISPLAY_BATTERY_GRAPH_MODE:
            instance->atoms3lcd.color_str = str.substring(1); // remove PacketType Header
            break;

        case SERVO_CONTROL_MODE:
            instance->atoms3lcd.color_str = str.substring(1); // remove PacketType Header
            break;

        case PRESSURE_CONTROL_MODE:
            instance->atoms3lcd.color_str = str.substring(1); // remove PacketType Header
            break;

        case TEACHING_MODE:
            instance->atoms3lcd.color_str = str.substring(1); // remove PacketType Header
            break;

        case DISPLAY_ODOM_MODE:
           instance->atoms3lcd.color_str = str.substring(1); // remove PacketType Header
           break;

        default:
            // Handle TEXT or unknown packets
            instance->atoms3lcd.color_str = str;
            break;
    }
}

void AtomS3I2C::handleJpegPacket(const String& str) {
    if (str.length() == 2 && !instance->atoms3lcd.readyJpeg) {
        // Initialize JPEG loading
        instance->atoms3lcd.jpegLength = (static_cast<uint32_t>(str[0]) << 8) | static_cast<uint8_t>(str[1]);
        instance->atoms3lcd.currentJpegIndex = 0;
        instance->atoms3lcd.loadingJpeg = true;
    } else if (instance->atoms3lcd.loadingJpeg) {
        size_t index = instance->atoms3lcd.currentJpegIndex;
        size_t strLength = str.length();
        // Continue loading JPEG data
        if (index + strLength <= sizeof(instance->atoms3lcd.jpegBuf)) {
          memcpy(instance->atoms3lcd.jpegBuf + index, str.c_str(), strLength);
            instance->atoms3lcd.currentJpegIndex += strLength;
        } else {
            instance->atoms3lcd.loadingJpeg = false;
        }
        if (instance->atoms3lcd.currentJpegIndex >= instance->atoms3lcd.jpegLength) {
            instance->atoms3lcd.loadingJpeg = false;
            instance->atoms3lcd.readyJpeg = true;
        }
    } else {
        instance->atoms3lcd.loadingJpeg = false;  // Reset if invalid packet received
    }
}

void AtomS3I2C::handleQrCodePacket(const String& str) {
    if (str.length() > 1) {
        uint8_t qrCodeLength = static_cast<uint8_t>(str[1]);
        instance->atoms3lcd.qrCodeData = str.substring(2, 2 + qrCodeLength);
    }
}

void AtomS3I2C::handleForceModePacket(const String& str) {
    if (str.length() > 1) {
        forcedMode = str.substring(1);
    }
}

void AtomS3I2C::handleSelectedModePacket(const String& str) {
    if (str.length() > 1) {
        selectedModesStr = str.substring(1);
    }
}

void AtomS3I2C::stopReceiveEvent() {
  receiveEventEnabled = false;
}

void AtomS3I2C::startReceiveEvent() {
  receiveEventEnabled = true;
}

void AtomS3I2C::requestEvent() {
  if (instance == nullptr)
      return;
  uint8_t sentStr[100];
  sentStr[0] = (uint8_t)instance->atoms3button.getButtonState();
  const char* modeData = requestStr.c_str();
 // sentStr[1]以降にstrDataをコピー (長さを確認)
  size_t strLen = strlen(modeData);  // requestStrの長さを取得
  if (strLen > 98) {
      strLen = 98;  // バッファオーバーフローを防ぐため最大98バイトに制限
  }
  memcpy(&sentStr[1], modeData, strLen);  // sentStr[1]以降にstrDataをコピー

  // WireSlave.print(requestStr);
  WireSlave.write(sentStr, strLen+1);

  instance->atoms3button.notChangedButtonState();
}

void AtomS3I2C::setRequestStr(const String &str) {
  requestStr = str;
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
