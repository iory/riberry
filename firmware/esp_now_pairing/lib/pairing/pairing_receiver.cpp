#include <pairing_receiver.h>

pairingData PairingReceiver::receivedData;

void PairingReceiver::setupESPNOW() {
  Pairing::setupESPNOW();

  // Broadcast receiver address to sender
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    statusStr = "Failed to add peer";
    return;
  }
  esp_now_register_recv_cb(onDataRecv);
  statusStr = "ESP-NOW init succeed.";
}

// Send receiver MAC address to sender
void PairingReceiver::broadcast() {
  uint8_t parity_data = 0;
  esp_now_send(broadcastAddress, &parity_data, sizeof(parity_data));
  statusStr = "Send MAC address on broadcast";
}

void PairingReceiver::onDataRecv(const uint8_t *mac_addr, const uint8_t *_receiveData, int data_len) {
  if(data_len == sizeof(pairingData)) {
    memcpy(&receivedData, _receiveData, data_len);
    char buf[50];
    sprintf(buf, "Receive IP Address\n %u.%u.%u.%u\n",
            receivedData.IPv4[0], receivedData.IPv4[1], receivedData.IPv4[2], receivedData.IPv4[3]);
    statusStr = String(buf);
    isESPNOWReceived = true;
  } else{
    statusStr = "Received data size does not match pairingData";
  }
}

pairingData PairingReceiver::getReceivedData() {
  return receivedData;
}

void PairingReceiver::sendDataToComputer(Stream& outputStream) {
  char buf[25];
  sprintf(buf, "%u.%u.%u.%u",
          receivedData.IPv4[0], receivedData.IPv4[1], receivedData.IPv4[2], receivedData.IPv4[3]);
  outputStream.println(String(buf));
}

void PairingReceiver::impl(Stream& outputStream) {
  if (isESPNOWReceived) {
    // Show send data result
    sendDataToComputer(outputStream);
  }
  else {
    broadcast();
    statusStr = "Send broadcast\nWait for sender to send pairing data...";
  }
}

String PairingReceiver::myRole() {
  return String("Receiver");
}

String PairingReceiver::basicInformation() {
  String info;
  info = "I am " + myRole() + "\n\n";
  info += "My MAC:\n " + getMyMACAddress();
  info += "\n\n" + statusStr;
  return info;
}
