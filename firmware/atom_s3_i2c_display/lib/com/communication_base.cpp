#include <communication_base.h>

CommunicationBase* CommunicationBase::instance = nullptr;
Stream* CommunicationBase::_stream = nullptr;
uint8_t CommunicationBase::requestBytes[100];
uint8_t CommunicationBase::forcedMode;
uint8_t CommunicationBase::selectedModesBytes[100];
Role CommunicationBase::role;
bool CommunicationBase::pairingEnabled = false;

CommunicationBase::CommunicationBase(
        PrimitiveLCD& lcd, ButtonManager& button, Pairing& pairing, Stream* stream, Role role)
    : lcd(lcd),
      button_manager(button),
      pairing(pairing),
      receiveEventEnabled(true),
      ExecutionTimer("CommunicationBase") {
    instance = this;
    setStream(stream);
    this->role = role;
}

void CommunicationBase::setStream(Stream* stream) { _stream = stream; }

Stream* CommunicationBase::getStream() const { return _stream; }

void CommunicationBase::updateLastReceiveTime() { lastReceiveTime = millis(); }

bool CommunicationBase::checkTimeout() { return millis() - lastReceiveTime > receiveTimeout; }

// Use char* instead of String for output[] argument
// to minimize dynamic memory usage and reduce the risk of fragmentation.
int CommunicationBase::splitString(const String& input,
                                   char delimiter,
                                   char* output[],
                                   int maxParts) {
    int start = 0;
    int index = 0;

    while (true) {
        int end = input.indexOf(delimiter, start);
        if (end == -1) {  // If the delimiter is not found
            if (index < maxParts) {
                String part = input.substring(start);
                output[index] = strdup(part.c_str());
                index++;
            }
            break;
        }
        if (index < maxParts) {
            String part = input.substring(start, end);
            output[index] = strdup(part.c_str());
            index++;
        }
        start = end + 1;
    }

    return index;  // Returns the number of divided portions
}

void CommunicationBase::receiveEvent(int howMany) {
    if (_stream == nullptr || instance == nullptr || !instance->receiveEventEnabled) return;

    instance->updateLastReceiveTime();
    String str;
    int offset = 0;

    if (_stream == &WireSlave) {
        while (_stream->available()) {
            char c = _stream->read();
            str += c;
        }
        offset = 1;
    } else {
        unsigned long start = millis();
        while (str.length() < 2) {
            if (_stream->available()) {
                char c = _stream->read();
                str += c;
            }
            if (millis() - start > PACKET_TIMEOUT) {
                return;
            }
            instance->delayWithTimeTracking(pdMS_TO_TICKS(1));
        }
        uint8_t packetType = (uint8_t)str.charAt(0);
        uint8_t packetLength = (uint8_t)str.charAt(1);

        start = millis();
        while (str.length() < packetLength) {
            if (_stream->available()) {
                char c = _stream->read();
                str += c;
            }
            if (millis() - start > PACKET_TIMEOUT) {
                return;
            }
            instance->delayWithTimeTracking(pdMS_TO_TICKS(1));
        }
        offset = 2;
    }

    if (str.length() < 1) {
        return;  // Invalid packet
    }

    PacketType packetType = static_cast<PacketType>(str[0]);
    switch (packetType) {
        case TEXT:
            instance->lcd.color_str = str.substring(offset);
            break;

        case JPEG:
            handleJpegPacket(str.substring(offset));
            break;

        case QR_CODE:
            handleQrCodePacket(str, offset);
            break;

        case FORCE_MODE:
            handleForceModePacket(str, offset);
            break;

        case SELECTED_MODE:
            handleSelectedModePacket(str, offset);
            break;

        case DISPLAY_BATTERY_GRAPH_MODE:
            instance->lcd.color_str = str.substring(offset);  // remove PacketType Header
            break;

        case SERVO_CONTROL_MODE:
            instance->lcd.color_str = str.substring(offset);  // remove PacketType Header
            break;

        case PRESSURE_CONTROL_MODE:
            instance->lcd.color_str = str.substring(offset);  // remove PacketType Header
            break;

        case TEACHING_MODE:
            instance->lcd.color_str = str.substring(offset);  // remove PacketType Header
            break;

        case DISPLAY_ODOM_MODE:
            instance->lcd.color_str = str.substring(offset);  // remove PacketType Header
            break;

        case SPEECH_TO_TEXT_MODE:
            instance->lcd.color_str = str.substring(offset);  // remove PacketType Header
            break;

        case BUTTON_STATE_REQUEST:
            requestEvent();
            break;

        case GET_PAIRING_TYPE:
            _stream->flush();
            if (pairingEnabled) {
                _stream->write(getRoleStr(role).c_str(), getRoleStr(role).length());
            }
            // If packets arrive while pairing is not enabled, return dummy data
            else {
                String dummy = "dummy";
                _stream->write(dummy.c_str(), dummy.length());
            }
            if (_stream == &WireSlave) {
                WireSlave.update();
            }
            break;

        case PAIRING_IP_REQUEST: {
            _stream->flush();
            if (pairingEnabled) {
                std::map<String, PairingData> pairedDataMap = instance->pairing.getPairedData();
                if (!pairedDataMap.empty()) {
                    auto it = pairedDataMap.begin();
                    _stream->write(it->second.IPv4, 4);
                    if (_stream == &WireSlave) {
                        WireSlave.update();
                    }
                }
                // If No paired data, return dummy IP address
                else {
                    uint8_t dummy_ip[4] = {255, 255, 255, 255};
                    _stream->write(dummy_ip, 4);
                    if (_stream == &WireSlave) {
                        WireSlave.update();
                    }
                }
            }
            // If packets arrive while pairing is not enabled, return dummy IP address
            else {
                uint8_t dummy_ip[4] = {255, 255, 255, 255};
                _stream->write(dummy_ip, 4);
                if (_stream == &WireSlave) {
                    WireSlave.update();
                }
            }
            break;
        }
        case SET_IP_REQUEST: {
            PairingData dataToSend;
            if (pairingEnabled) {
                for (int i = 0; i < 4; i++) {
                    dataToSend.IPv4[i] = str[i + 1];
                }
            }
            // If packets arrive while pairing is not enabled, return dummy data
            else {
                for (int i = 0; i < 4; i++) {
                    dataToSend.IPv4[i] = 255;
                }
            }
            instance->pairing.setDataToSend(dataToSend);
            break;
        }
        default:
            // unknown packets
            instance->lcd.color_str = str;
            break;
    }
}

void CommunicationBase::handleJpegPacket(const String& str) {
    if (str.length() == 2 && !instance->lcd.readyJpeg) {
        // Initialize JPEG loading
        instance->lcd.jpegLength =
                (static_cast<uint32_t>(str[0]) << 8) | static_cast<uint8_t>(str[1]);
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

void CommunicationBase::handleQrCodePacket(const String& str, int offset) {
    if (str.length() > offset) {
        uint8_t qrCodeLength = static_cast<uint8_t>(str[offset]);
        instance->lcd.qrCodeData = str.substring(offset + 1, offset + 1 + qrCodeLength);
    }
}

void CommunicationBase::handleForceModePacket(const String& str, int offset) {
    if (str.length() > offset) {
        String substring = str.substring(offset);
        const uint8_t* buff = (const uint8_t*)substring.c_str();
        forcedMode = buff[0];
    }
}

void CommunicationBase::handleSelectedModePacket(const String& str, int offset) {
    if (str.length() > offset) {
        String substring = str.substring(offset);
        const uint8_t* buff = (const uint8_t*)substring.c_str();
        size_t byteLen = buff[0];
        memcpy(selectedModesBytes, buff, byteLen);
    }
}

void CommunicationBase::stopReceiveEvent() { receiveEventEnabled = false; }

void CommunicationBase::startReceiveEvent() { receiveEventEnabled = true; }

void CommunicationBase::requestEvent() {
    if (_stream == nullptr || instance == nullptr) return;
    size_t byteLen = requestBytes[0];
    requestBytes[1] = (uint8_t)instance->button_manager.getButtonState();
    _stream->write(requestBytes, byteLen);
    instance->button_manager.notChangedButtonState();
}

void CommunicationBase::setRequestBytes(uint8_t* bytes, size_t byteLen) {
    requestBytes[0] = byteLen + 2;
    // requestBytes[1] is button information
    memcpy(&requestBytes[2], bytes, byteLen);  // Mode information
}

void CommunicationBase::task(void* parameter) {
    if (_stream == nullptr) {
        instance->lcd.printColorText("Stream not initialized\n");
        return;
    }

    if (_stream == &WireSlave) {
        instance->lastReceiveTime = millis() - instance->receiveTimeout;
        WireSlave.onReceive(receiveEvent);

        while (true) {
            WireSlave.update();
            // After increasing the delay time after WireSlave.update() from 1ms to 10ms, the CPU
            // usage dropped from around 60% to 16%. If the CPU usage on Core 0 becomes too high, we
            // should consider reducing this value accordingly.
            instance->delayWithTimeTracking(pdMS_TO_TICKS(1));
        }
#if ARDUINO_USB_MODE
    #if ARDUINO_USB_CDC_ON_BOOT  // Serial used for USB CDC
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
            // Without this delay, the task could block other lower-priority
            // tasks or prevent the watchdog timer from resetting in time,
            // causing a "Task watchdog got triggered" error.
            instance->delayWithTimeTracking(pdMS_TO_TICKS(1));
        }
    } else {
        instance->lcd.printColorText("Unsupported Stream type\n");
        return;
    }
}

void CommunicationBase::createTask(uint8_t xCoreID) {
    this->xCoreID = xCoreID;
    xTaskCreatePinnedToCore(task, "I2C Task", 2048, this, 24, NULL, xCoreID);
}
