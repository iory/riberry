#include <communication_base.h>

CommunicationBase* CommunicationBase::instance = nullptr;
String CommunicationBase::requestStr = ""; // Initialize the static requestStr
String CommunicationBase::forcedMode = ""; // Initialize the static forcedMode
String CommunicationBase::selectedModesStr = ""; // Initialize the static selectedModesStr

CommunicationBase::CommunicationBase(AtomS3LCD &lcd, ButtonManager &button)
  : atoms3lcd(lcd), button_manager(button), receiveEventEnabled(true) {
  instance = this;
}

void CommunicationBase::updateLastReceiveTime() {
  lastReceiveTime = millis();
}

bool CommunicationBase::checkTimeout() {
  return millis() - lastReceiveTime > receiveTimeout;
}

// Use char* instead of String for output[] argument
// to minimize dynamic memory usage and reduce the risk of fragmentation.
int CommunicationBase::splitString(const String &input, char delimiter, char* output[], int maxParts) {
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


void CommunicationBase::receiveEvent(int howMany) {
  if (instance->receiveEventEnabled == false)
    return;
  if (instance == nullptr)
    return;
  instance->updateLastReceiveTime();
  String str;
  while (0 < Serial.available()) {
    char c = Serial.read();
    str += c;
  }
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

  case BUTTON_STATE_REQUEST:
    requestEvent();
    break;

  default:
    // Handle TEXT or unknown packets
    instance->atoms3lcd.color_str = str;
    break;
  }
}

void CommunicationBase::handleJpegPacket(const String& str) {
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

void CommunicationBase::handleQrCodePacket(const String& str) {
    if (str.length() > 1) {
        uint8_t qrCodeLength = static_cast<uint8_t>(str[1]);
        instance->atoms3lcd.qrCodeData = str.substring(2, 2 + qrCodeLength);
    }
}

void CommunicationBase::handleForceModePacket(const String& str) {
    if (str.length() > 1) {
        forcedMode = str.substring(1);
    }
}

void CommunicationBase::handleSelectedModePacket(const String& str) {
    if (str.length() > 1) {
        selectedModesStr = str.substring(1);
    }
}

void CommunicationBase::stopReceiveEvent() {
  receiveEventEnabled = false;
}

void CommunicationBase::startReceiveEvent() {
  receiveEventEnabled = true;
}

void CommunicationBase::requestEvent() {
  if (instance == nullptr)
      return;
  uint8_t sentStr[100];
  sentStr[0] = (uint8_t)instance->button_manager.getButtonState();
  const char* modeData = requestStr.c_str();
 // sentStr[1]以降にstrDataをコピー (長さを確認)
  size_t strLen = strlen(modeData);  // requestStrの長さを取得
  if (strLen > 98) {
      strLen = 98;  // バッファオーバーフローを防ぐため最大98バイトに制限
  }
  memcpy(&sentStr[1], modeData, strLen);  // sentStr[1]以降にstrDataをコピー

#ifdef ATOM_S3
  WireSlave.write(sentStr, strLen+1);
#elif defined(USE_M5STACK_BASIC)
  Serial.write(sentStr, strLen + 1);
#endif
  instance->button_manager.notChangedButtonState();
}

void CommunicationBase::setRequestStr(const String &str) {
  requestStr = str;
}

void CommunicationBase::task(void *parameter) {
#ifdef ATOM_S3
  bool success = WireSlave.begin(sda_pin, scl_pin, i2c_slave_addr, 200, 100);
  if (!success) {
    instance->atoms3lcd.printColorText("I2C slave init failed\n");
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
#elif defined(USE_M5STACK_BASIC)
  Serial.begin(115200, SERIAL_8N1, 16, 17);
  instance->lastReceiveTime = millis() - instance->receiveTimeout;
  while (true) {
    if (Serial.available() > 0) {
      receiveEvent(Serial.available());
    }
  }
#endif
}

void CommunicationBase::createTask(uint8_t xCoreID) {
  xTaskCreatePinnedToCore(task, "I2C Task", 2048, this, 24, NULL, xCoreID);
}
