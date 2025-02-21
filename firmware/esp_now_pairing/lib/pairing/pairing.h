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
    void waitForESPNOWSetup();

    void sendPairingData(const PairingData& data);
    static bool receivePairingData(String macString, PairingData& receivedData);
    void broadcastMACAddress();
    static bool addPeer(const String& macAddress);
    void checkPendingPeers();

    String getMyMACAddress() const;
    String getStatus() const;
    void reset();

    static bool isPairingActive() { return _pairingActive; }
    static void startPairing();
    static void stopPairing();
    bool isPaired() const { return pairedMACAddresses.size() > 0; }
    std::vector<String> getPairedMACAddresses() const { return pairedMACAddresses; }
    std::map<String, PairingData> getPairedData() const { return pairingDataMap; }

    void setupBroadcastPeer();
    void createTask(uint8_t xCoreID);
    void task();
    void handleSuspend(bool* resumeAfterSuspend);
    void resumeTask(uint8_t xCoreID);
    void suspendTask();
    void deleteTask();

    void setDataToSend(const PairingData& data);

private:
    static MutexHelper mutex_;

    // EspNow callbacks
    static void onDataSent(const uint8_t* mac_addr, esp_now_send_status_t status);
    static void onDataRecv(const uint8_t* mac_addr, const uint8_t* data, int data_len);

    // Mac addresses and Pairing Information
    uint8_t myMACAddress[6];
    static std::map<String, PairingData> pairingDataMap;
    static std::map<String, unsigned long> pendingPeers;
    static std::vector<String> pairedMACAddresses;
    const uint8_t broadcastAddress[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    // Task related variables
    TaskHandle_t taskHandle = nullptr;
    bool suspend = false;

    // status
    static String statusStr;
    static bool _pairingActive;
    const unsigned long pairingTimeout = 5000;

    PairingData dataToSend;
    bool dataToSendInitialized = false;
};

#endif  // PAIRING_H
