#include <communication_base.h>

CommunicationBase* CommunicationBase::instance = nullptr;
String CommunicationBase::requestStr = ""; // Initialize the static requestStr
String CommunicationBase::forcedMode = ""; // Initialize the static forcedMode
String CommunicationBase::selectedModesStr = ""; // Initialize the static selectedModesStr

CommunicationBase::CommunicationBase(PrimitiveLCD &lcd, ButtonManagers &buttons)
  : lcd(lcd), button_managers(buttons), receiveEventEnabled(true) {
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
#ifdef ATOM_S3
  // Read data from the I2C bus
  while (0 < WireSlave.available()) {
    char c = WireSlave.read();  // receive byte as a character;
    str += c;
  }
#elif defined(USE_M5STACK_BASIC)
  while (0 < Serial.available()) {
    char c = Serial.read();  // receive byte as a character;
    str += c;
  }
#endif
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
    instance->lcd.color_str = str.substring(1); // remove PacketType Header
    break;

  case SERVO_CONTROL_MODE:
    instance->lcd.color_str = str.substring(1); // remove PacketType Header
    break;

  case PRESSURE_CONTROL_MODE:
    instance->lcd.color_str = str.substring(1); // remove PacketType Header
    break;

  case TEACHING_MODE:
    instance->lcd.color_str = str.substring(1); // remove PacketType Header
    break;

  case DISPLAY_ODOM_MODE:
    instance->lcd.color_str = str.substring(1); // remove PacketType Header
    break;

  case BUTTON_STATE_REQUEST:
    requestEvent();
    break;

  default:
    // Handle TEXT or unknown packets
    instance->lcd.color_str = str;
    break;
  }
}

void CommunicationBase::handleJpegPacket(const String& str) {
    if (str.length() == 2 && !instance->lcd.readyJpeg) {
        // Initialize JPEG loading
        instance->lcd.jpegLength = (static_cast<uint32_t>(str[0]) << 8) | static_cast<uint8_t>(str[1]);
        instance->lcd.currentJpegIndex = 0;
        instance->lcd.loadingJpeg = true;
    } else if (instance->lcd.loadingJpeg) {
        size_t index = instance->lcd.currentJpegIndex;
        size_t strLength = str.length();
        // Continue loading JPEG data
        if (index + strLength <= sizeof(instance->lcd.jpegBuf)) {
          memcpy(instance->lcd.jpegBuf + index, str.c_str(), strLength);
            instance->lcd.currentJpegIndex += strLength;
        } else {
            instance->lcd.loadingJpeg = false;
        }
        if (instance->lcd.currentJpegIndex >= instance->lcd.jpegLength) {
            instance->lcd.loadingJpeg = false;
            instance->lcd.readyJpeg = true;
        }
    } else {
        instance->lcd.loadingJpeg = false;  // Reset if invalid packet received
    }
}

void CommunicationBase::handleQrCodePacket(const String& str) {
    if (str.length() > 1) {
        uint8_t qrCodeLength = static_cast<uint8_t>(str[1]);
        instance->lcd.qrCodeData = str.substring(2, 2 + qrCodeLength);
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
  size_t numButton = instance->button_managers.getNumberOfButtons();
  size_t packetOffset = 1;
  sentStr[0] = numButton;
  for (size_t i = 0; i < numButton; ++i) {
      sentStr[packetOffset + i] = (uint8_t)instance->button_managers.getButtonState(i);
      instance->button_managers.resetButtonState(i);
  }
  const char* modeData = requestStr.c_str();
 // sentStr[1]以降にstrDataをコピー (長さを確認)
  size_t strLen = strlen(modeData);  // requestStrの長さを取得
  if (strLen > 98) {
      strLen = 98;  // バッファオーバーフローを防ぐため最大98バイトに制限
  }
  memcpy(&sentStr[packetOffset + numButton], modeData, strLen);  // sentStr[1]以降にstrDataをコピー

#ifdef ATOM_S3
  WireSlave.write(sentStr, strLen+1);
#elif defined(USE_M5STACK_BASIC)
  Serial.write(sentStr, strLen + 1);
#endif
}

void CommunicationBase::setRequestStr(const String &str) {
  requestStr = str;
}

void CommunicationBase::task(void *parameter) {
#ifdef ATOM_S3
  bool success = WireSlave.begin(sda_pin, scl_pin, i2c_slave_addr, 200, 100);
  if (!success) {
    instance->lcd.printColorText("I2C slave init failed\n");
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
    // Insert a short delay to yield control to the RTOS scheduler.
    // Without this delay, the task could block other lower-priority tasks
    // or prevent the watchdog timer from resetting in time, causing a "Task watchdog got triggered" error.
    vTaskDelay(pdMS_TO_TICKS(1));
  }
#endif
}

void CommunicationBase::createTask(uint8_t xCoreID) {
  xTaskCreatePinnedToCore(task, "I2C Task", 2048, this, 24, NULL, xCoreID);
}
