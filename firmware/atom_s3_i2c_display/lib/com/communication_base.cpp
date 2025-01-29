#include <communication_base.h>

CommunicationBase* CommunicationBase::instance = nullptr;
Stream* CommunicationBase::_stream = nullptr;
String CommunicationBase::requestStr = ""; // Initialize the static requestStr
String CommunicationBase::forcedMode = ""; // Initialize the static forcedMode
String CommunicationBase::selectedModesStr = ""; // Initialize the static selectedModesStr
String CommunicationBase::main_or_secondary = "";

CommunicationBase::CommunicationBase(PrimitiveLCD &lcd, ButtonManager &button, Pairing &pairing, Stream* stream, String main_or_secondary)
  : lcd(lcd), button_manager(button), pairing(pairing), receiveEventEnabled(true) {
  instance = this;
  setStream(stream);
  this->main_or_secondary = main_or_secondary;
}

void CommunicationBase::setStream(Stream* stream) {
  _stream = stream;
}

Stream* CommunicationBase::getStream() const {
  return _stream;
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
  if (_stream == nullptr || instance == nullptr || !instance->receiveEventEnabled)
    return;

  instance->updateLastReceiveTime();
  String str;

  while (_stream->available()) {
    char c = _stream->read();
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

  case GET_PAIRING_TYPE:
    _stream->write(main_or_secondary.c_str(), main_or_secondary.length());
    break;

  case PAIRING_IP_REQUEST: {
    std::map<String, PairingData> pairedDataMap = instance->pairing.getPairedData();
    if (!pairedDataMap.empty()) {
      auto it = pairedDataMap.begin();
      _stream->write(it->second.IPv4[0]);
      _stream->write(it->second.IPv4[1]);
      _stream->write(it->second.IPv4[2]);
      _stream->write(it->second.IPv4[3]);
    }
    break;
  }
  case SET_IP_REQUEST: {
    PairingData dataToSend;
    for (int i = 0; i < 4; i++) {
      dataToSend.IPv4[i] = str[i + 1];
    }
    instance->pairing.setDataToSend(dataToSend);
    break;
  }
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
  if (_stream == nullptr || instance == nullptr)
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

  _stream->write(sentStr, strLen + 1);
  instance->button_manager.notChangedButtonState();
}

void CommunicationBase::setRequestStr(const String &str) {
  requestStr = str;
}

void CommunicationBase::task(void *parameter) {
  if (_stream == nullptr) {
    instance->lcd.printColorText("Stream not initialized\n");
    return;
  }

  if (_stream == &WireSlave) {
    instance->lastReceiveTime = millis() - instance->receiveTimeout;
    WireSlave.onReceive(receiveEvent);
    WireSlave.onRequest(requestEvent);

    while (true) {
      WireSlave.update();
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  #if ARDUINO_USB_MODE
  #if ARDUINO_USB_CDC_ON_BOOT // Serial used for USB CDC
    } else if (_stream == &Serial || _stream == &Serial1) {
  #else
    } else if (_stream == &Serial || _stream == &Serial1 || _stream == &USBSerial) {
  #endif
  #else
    } else if (_stream == &Serial || _stream == &Serial1) {
  #endif
    instance->lastReceiveTime = millis() - instance->receiveTimeout;

    while (true) {
      if (_stream->available() > 0) {
        receiveEvent(_stream->available());
      }
      // Insert a short delay to yield control to the RTOS scheduler.
      // Without this delay, the task could block other lower-priority tasks
      // or prevent the watchdog timer from resetting in time, causing a "Task watchdog got triggered" error.
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  } else {
    instance->lcd.printColorText("Unsupported Stream type\n");
    return;
  }
}

void CommunicationBase::createTask(uint8_t xCoreID) {
  xTaskCreatePinnedToCore(task, "I2C Task", 2048, this, 24, NULL, xCoreID);
}
