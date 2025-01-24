#ifndef PAIRING_RECEIVER_H
#define PAIRING_RECEIVER_H

#include <pairing.h>

class PairingReceiver : public Pairing {
public:
  void setupESPNOW() override;
  PairingReceiver() : Pairing() {}
  pairingData getReceivedData();
  void impl(Stream& outputStream);
  void broadcast();
  String myRole() override;
  String basicInformation() override;
private:
  static pairingData receivedData;
  static void onDataRecv(const uint8_t *mac_addr, const uint8_t *_receiveData, int data_len);
  void sendDataToComputer(Stream& outputStream);
};

#endif // PAIRING_RECEIVER_H
