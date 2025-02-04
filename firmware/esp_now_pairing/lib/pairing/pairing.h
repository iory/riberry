#ifndef PAIRING_H
#define PAIRING_H

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include <map>
#include <vector>

#include "mutex_helper.h"

struct PairingData {
    uint8_t IPv4[4];
};

class Pairing {
public:
    Pairing();
    bool setupESPNOW();
    void sendPairingData(const PairingData& data);
    void setupBroadcastPeer();
    static bool receivePairingData(String macString, PairingData& receivedData);
    void broadcastMACAddress();
    String getMyMACAddress() const;
    String getStatus() const;
    void reset();
    static bool isPairingActive() { return _pairingActive; }
    static void stopPairing() {
        mutex_.lock();
        _pairingActive = false;
        mutex_.unlock();
    }
    static void startPairing() {
        mutex_.lock();
        _pairingActive = true;
        mutex_.unlock();
    }
    bool isPaired() const { return pairedMACAddresses.size() > 0; }
    std::vector<String> getPairedMACAddresses() const { return pairedMACAddresses; }
    std::map<String, PairingData> getPairedData() const { return pairingDataMap; }
    static bool addPeer(const String& macAddress);
    void checkPendingPeers();
    void startBackgroundTask(uint8_t xCoreID) {
        if (taskHandle == nullptr) {
            xTaskCreatePinnedToCore([](void* _this) { static_cast<Pairing*>(_this)->task(); },
                                    "Pairing Task", 2048, this, 1, &taskHandle, xCoreID);
        }
    }

    void stopBackgroundTask() {
        if (taskHandle != nullptr) {
            vTaskSuspend(taskHandle);
        }
    }

    void deleteTask() {
        if (taskHandle != nullptr) {
            TaskHandle_t taskToDelete = taskHandle;
            vTaskDelete(taskToDelete);
            while (eTaskGetState(taskToDelete) != eDeleted) {
                vTaskDelay(1 / portTICK_PERIOD_MS);
            }
            taskHandle = nullptr;
        }
    }
    void resumeBackgroundTask() {
        if (taskHandle != nullptr) {
            vTaskResume(taskHandle);
        }
    }

    void setDataToSend(const PairingData& data) {
        dataToSend = data;
        dataToSendInitialized = true;
    }

private:
    static MutexHelper mutex_;
    static void onDataSent(const uint8_t* mac_addr, esp_now_send_status_t status);
    static void onDataRecv(const uint8_t* mac_addr, const uint8_t* data, int data_len);
    uint8_t myMACAddress[6];
    const uint8_t broadcastAddress[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_peer_info_t peerInfo;
    static String statusStr;
    static std::map<String, PairingData> pairingDataMap;
    static std::map<String, unsigned long> pendingPeers;
    static std::vector<String> pairedMACAddresses;
    const unsigned long pairingTimeout = 5000;
    PairingData dataToSend;
    bool dataToSendInitialized = false;
    static bool _pairingActive;
    TaskHandle_t taskHandle = nullptr;

    void task() {
        while (!setupESPNOW()) {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        for (;;) {
            checkPendingPeers();
            if (_pairingActive) {
                broadcastMACAddress();
            }
            if (isPaired() && dataToSendInitialized) {
                sendPairingData(dataToSend);
            }
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
};

#endif  // PAIRING_H
