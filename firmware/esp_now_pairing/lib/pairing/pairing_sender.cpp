#include <pairing_sender.h>

uint8_t PairingSender::receiverMACAddress[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}; // dummy

PairingSender::PairingSender() {
  new Pairing();

  if (getMyMACAddress().equals(getReceiverMACAddress())) {
    statusStr = "Target MAC address is same as my MAC Address:\n " + getMyMACAddress();
  }
}

void PairingSender::setupESPNOW() {
  Pairing::setupESPNOW();
  esp_now_register_recv_cb(onDataRecv);
  statusStr = "ESP-NOW init succeed\nWaiting for broadcast from receiver...";
}

String PairingSender::getReceiverMACAddress() {
  // Show my MAC address
  char buf[25];
  uint8_t* addr = receiverMACAddress;
  sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X",
          addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
  return String(buf);
}

void PairingSender::sendPairingData() {
  esp_now_send(receiverMACAddress,
               (uint8_t*)&pairingDataFromComputer, sizeof(pairingDataFromComputer));
  char buf[40];
  sprintf(buf, "Send IP Address\n %u.%u.%u.%u\n",
          pairingDataFromComputer.IPv4[0], pairingDataFromComputer.IPv4[1],
          pairingDataFromComputer.IPv4[2], pairingDataFromComputer.IPv4[3]);
  statusStr = String(buf);
}

// This function is called after data is sent by ESP-NOW
// Check if the transmission was successful or not.
void PairingSender::onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    statusStr = "Waiting for ESP-NOW peer to send data";
  }
}

void PairingSender::onDataRecv(const uint8_t *mac_addr, const uint8_t *_receiveData, int data_len) {
  if (isESPNOWReceived)
    return;
  // length and data check
  if (data_len != sizeof(uint8_t) || *_receiveData != 0)
    return;
  isESPNOWReceived = true;

  // Register receiver mac address
  memcpy(receiverMACAddress, mac_addr, 6);
  memcpy(peerInfo.peer_addr, receiverMACAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    statusStr = "Failed to add peer";
    return;
  }
  esp_now_register_send_cb(onDataSent);
  // Set statusStr
  char buf[60];
  sprintf(buf, "Register receiver MAC address:\n %02X:%02X:%02X:%02X:%02X:%02X",
          receiverMACAddress[0], receiverMACAddress[1], receiverMACAddress[2],
          receiverMACAddress[3], receiverMACAddress[4], receiverMACAddress[5]);
  statusStr = String(buf);
}

String PairingSender::myRole() {
  return String("Sender");
}

void PairingSender::impl(Stream& outputStream) {
  // Show send data result
  sendPairingData();
}

String PairingSender::basicInformation() {
  String info;
  info = "I am " + myRole() + "\n\n";
  info += "My MAC:\n " + getMyMACAddress();
  info += "Receiver MAC:\n " + getReceiverMACAddress();
  info += "\n\n" + statusStr;
  return info;
}
