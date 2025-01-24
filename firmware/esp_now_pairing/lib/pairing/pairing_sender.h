#ifndef PAIRING_SENDER_H
#define PAIRING_SENDER_H

#include <pairing.h>

class PairingSender : public Pairing {
public:
  void setupESPNOW() override;
  void impl(Stream& outputStream) override;
  PairingSender();
  String getReceiverMACAddress();
  void sendPairingData();
  String myRole() override;
  String basicInformation() override;
private:
  // MAC address of pairing receiver
  static uint8_t receiverMACAddress[6];
  static void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
  static void onDataRecv(const uint8_t *mac_addr, const uint8_t *_receiveData, int data_len);
};

#endif // PAIRING_SENDER_H
