#include "pairing.h"

std::map<String, unsigned long> Pairing::pendingPeers = {};
String Pairing::statusStr = "";
bool Pairing::_pairingActive = true;
std::vector<String> Pairing::pairedMACAddresses = {};
std::map<String, PairingData> Pairing::pairingDataMap = {};
MutexHelper Pairing::mutex_;

Pairing::Pairing() : ExecutionTimer("Pairing") { esp_read_mac(myMACAddress, ESP_MAC_WIFI_STA); }

bool Pairing::setupESPNOW() {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_MODE_STA);
    if (esp_now_init() != ESP_OK) {
        statusStr = "Error initializing ESP-NOW";
        return false;
    }
    setupBroadcastPeer();
    esp_now_register_recv_cb(onDataRecv);
    esp_now_register_send_cb(onDataSent);
    statusStr = "ESP-NOW initialized successfully";
    return true;
}

void Pairing::setupBroadcastPeer() {
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(esp_now_peer_info_t));
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        statusStr = "Failed to add broadcast peer";
    } else {
        statusStr = "Broadcast peer added successfully";
    }
}

bool Pairing::addPeer(const String& macAddress) {
    uint8_t peerMACAddress[6];
    int result = sscanf(macAddress.c_str(), "%02X:%02X:%02X:%02X:%02X:%02X", &peerMACAddress[0],
                        &peerMACAddress[1], &peerMACAddress[2], &peerMACAddress[3],
                        &peerMACAddress[4], &peerMACAddress[5]);
    if (result != 6) {
        statusStr = "Error parsing MAC address in addPeer: " + macAddress;
        return false;
    }

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, peerMACAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_is_peer_exist(peerMACAddress)) {
        statusStr = "Peer already exists: " + macAddress;
        return true;
    }
    esp_err_t addResult = esp_now_add_peer(&peerInfo);
    if (addResult == ESP_OK) {
        statusStr = "Peer added successfully: " + macAddress;
        return true;
    } else {
        statusStr = "Failed to add peer: " + macAddress + " Error code: " + String(addResult) +
                    " Error message: " + esp_err_to_name(addResult);
        return false;
    }
}

void Pairing::sendPairingData(const PairingData& data) {
    mutex_.lock();
    for (const auto& macAddress : pairedMACAddresses) {
        uint8_t peerMACAddress[6];

        statusStr = "Sending data to: " + macAddress;
        int result = sscanf(macAddress.c_str(), "%02X:%02X:%02X:%02X:%02X:%02X", &peerMACAddress[0],
                            &peerMACAddress[1], &peerMACAddress[2], &peerMACAddress[3],
                            &peerMACAddress[4], &peerMACAddress[5]);

        if (result != 6) {
            statusStr = "Error parsing MAC address: " + macAddress;
            continue;
        }

        // To avoid EXCCAUSE: 0x0000001c (LoadStoreAlignmentCause), data must be cast to (uint8_t*)
        esp_err_t resultSend = esp_now_send(peerMACAddress, (uint8_t*)&data, sizeof(data));
        if (resultSend == ESP_OK) {
            statusStr = "Data sent successfully to: " + macAddress;
        } else {
            statusStr = "Failed to send data to: " + macAddress +
                        " Error code: " + String(resultSend) +
                        " Error message: " + esp_err_to_name(resultSend);
        }
    }
    mutex_.unlock();
    statusStr = "Data sent to all paired devices";
}

bool Pairing::receivePairingData(String macString, PairingData& receivedData) {
    if (pairedMACAddresses.size() == 0) {
        statusStr = "No data received yet";
        return false;
    }
    mutex_.lock();
    pairingDataMap[macString] = receivedData;
    mutex_.unlock();
    statusStr = "Data received successfully from: " + macString;
    return true;
}

void Pairing::broadcastMACAddress() {
    uint8_t pairingRequest = 0x01;
    // To avoid EXCCAUSE: 0x0000001c (LoadStoreAlignmentCause), data must be cast to (uint8_t*)
    esp_err_t result =
            esp_now_send(broadcastAddress, (uint8_t*)&pairingRequest, sizeof(pairingRequest));
    if (result == ESP_OK) {
        statusStr = "Broadcasting MAC Address";
    } else {
        statusStr = "esp_now_send failed. Error code: " + String(result) +
                    " Error message: " + esp_err_to_name(result);
    }
}

String Pairing::getMyMACAddress() const {
    char buf[25];
    sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X", myMACAddress[0], myMACAddress[1], myMACAddress[2],
            myMACAddress[3], myMACAddress[4], myMACAddress[5]);
    return String(buf);
}

String Pairing::getStatus() const { return statusStr; }

void Pairing::onDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {}

void Pairing::checkPendingPeers() {
    unsigned long currentTime = millis();
    mutex_.lock();
    for (auto it = pendingPeers.begin(); it != pendingPeers.end();) {
        if (currentTime - it->second > pairingTimeout) {
            statusStr = "Pairing request timed out for: " + it->first;
            it = pendingPeers.erase(it);
        } else {
            ++it;
        }
    }
    mutex_.unlock();
}

void Pairing::onDataRecv(const uint8_t* mac_addr, const uint8_t* data, int data_len) {
    char macStr[18];
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac_addr[0], mac_addr[1], mac_addr[2],
            mac_addr[3], mac_addr[4], mac_addr[5]);
    String macString = String(macStr);

    statusStr = "Received data from: " + macString + " Data length: " + String(data_len);

    if (_pairingActive && data_len == 1 && data[0] == 0x01) {
        statusStr = "Pairing request received from: " + macString;
        uint8_t pairingResponse = 0x02;
        if (addPeer(macString) == false) {
            return;
        }
        // To avoid EXCCAUSE: 0x0000001c (LoadStoreAlignmentCause), data must be cast to (uint8_t*)
        if (esp_now_send(mac_addr, (uint8_t*)&pairingResponse, sizeof(pairingResponse)) != ESP_OK) {
            return;
        }
        mutex_.lock();
        pendingPeers[macString] = millis();
        mutex_.unlock();
        statusStr = "Pairing response sent to: " + macString;

    } else if (_pairingActive && data_len == 1 && data[0] == 0x02) {
        statusStr = "Pairing response received from: " + macString;
        if (std::find(pairedMACAddresses.begin(), pairedMACAddresses.end(), macString) ==
            pairedMACAddresses.end()) {
            if (addPeer(macString) == false) {
                return;
            }
            mutex_.lock();
            pairedMACAddresses.push_back(macString);
            mutex_.unlock();
            statusStr = "Added to paired list: " + macString;
        }
        statusStr = "Pairing complete with: " + macString;

        mutex_.lock();
        if (pendingPeers.find(macString) != pendingPeers.end()) {
            pendingPeers.erase(macString);
        }
        mutex_.unlock();
    } else if (pairedMACAddresses.size() > 0 && data_len == sizeof(PairingData)) {
        PairingData receivedData;
        memcpy(&receivedData, data, data_len);
        receivePairingData(macString, receivedData);
    }
}

void Pairing::reset() {
    mutex_.lock();
    pairedMACAddresses.clear();
    pairingDataMap.clear();
    pendingPeers.clear();
    mutex_.unlock();
}

void Pairing::createTask(uint8_t xCoreID) {
    if (taskHandle == nullptr) {
        this->xCoreID = xCoreID;
        xTaskCreatePinnedToCore([](void* _this) { static_cast<Pairing*>(_this)->task(); },
                                "Pairing Task", 4096, this, 1, &taskHandle, xCoreID);
    }
}

void Pairing::resumeTask(uint8_t xCoreID) {
    if (taskHandle == nullptr) {
        createTask(xCoreID);
    } else if (taskHandle != nullptr) {
        vTaskResume(taskHandle);
    }
    suspend = false;
}

void Pairing::suspendTask() {
    if (taskHandle != nullptr) {
        suspend = true;
        delay(100);
        vTaskSuspend(taskHandle);
    }
}

void Pairing::deleteTask() {
    if (taskHandle != nullptr) {
        TaskHandle_t taskToDelete = taskHandle;
        vTaskDelete(taskToDelete);
        while (eTaskGetState(taskToDelete) != eDeleted) {
            delayWithTimeTracking(1 / portTICK_PERIOD_MS);
        }
        taskHandle = nullptr;
        this->xCoreID = -1;
    }
}

void Pairing::setDataToSend(const PairingData& data) {
    dataToSend = data;
    dataToSendInitialized = true;
}

void Pairing::waitForESPNOWSetup() {
    while (!setupESPNOW()) {
        delayWithTimeTracking(1000 / portTICK_PERIOD_MS);
    }
}

void Pairing::handleSuspend(bool* resumeAfterSuspend) {
    if (suspend) {
        *resumeAfterSuspend = true;
        delayWithTimeTracking(100 / portTICK_PERIOD_MS);
    } else if (*resumeAfterSuspend) {
        *resumeAfterSuspend = false;
        waitForESPNOWSetup();
    }
}

void Pairing::task() {
    waitForESPNOWSetup();
    bool resumeAfterSuspend = false;

    for (;;) {
        handleSuspend(&resumeAfterSuspend);
        if (suspend) continue;

        checkPendingPeers();
        if (_pairingActive) {
            broadcastMACAddress();
        }
        if (isPaired() && dataToSendInitialized) {
            sendPairingData(dataToSend);
        }
        delayWithTimeTracking(100 / portTICK_PERIOD_MS);
    }
}

void Pairing::startPairing() {
    mutex_.lock();
    _pairingActive = true;
    mutex_.unlock();
}

void Pairing::stopPairing() {
    mutex_.lock();
    _pairingActive = false;
    mutex_.unlock();
}
