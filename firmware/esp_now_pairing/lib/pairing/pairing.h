#ifndef PAIRING_H
#define PAIRING_H

#include <esp_now.h>
#include <WiFi.h>

// the maximum data size that can be sent by ESP-Now is 250 bytes.
struct pairingData{
  uint8_t IPv4[4];
};

class Pairing {
public:
  Pairing();
  virtual void setupESPNOW();
  virtual void impl(Stream& outputStream);
  virtual String basicInformation();
  bool receivePairingData(const String& ipAddress);
  virtual String myRole();
protected:
  static bool isESPNOWReceived;
  static String statusStr;
  pairingData pairingDataFromComputer;
  bool initialized;
  uint8_t broadcastAddress[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  static esp_now_peer_info_t peerInfo;
  String getMyMACAddress();
private:
  uint8_t myMACAddress[6];
};

int splitString(const String &input, char delimiter, char* output[], int maxParts);

#endif // PAIRING_H
