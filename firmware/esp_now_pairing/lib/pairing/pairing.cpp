#include <pairing.h>

esp_now_peer_info_t Pairing::peerInfo;
String Pairing::statusStr = "Waiting for USB connection";
bool Pairing::isESPNOWReceived = false;

Pairing::Pairing() : initialized(false) {
  esp_read_mac(myMACAddress, ESP_MAC_WIFI_STA);
}

void Pairing::setupESPNOW() {
  // Setup ESP-NOW conection
  WiFi.mode(WIFI_MODE_STA);
  if(esp_now_init() != ESP_OK){
    statusStr = "Error initializing ESP-NOW";
    return;
  }
}

void Pairing::impl(Stream& outputStream) {
}

String Pairing::getMyMACAddress() {
  // Show my MAC address
  char buf[25];
  uint8_t* addr = myMACAddress;
  sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X",
          addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
  return String(buf);
}

bool Pairing::receivePairingData(const String& ipAddress) {
  String trimmedIPAddress = ipAddress;
  trimmedIPAddress.trim(); // Remove newline

  // Set pairing data
  int listSize = 4;
  char** strList = (char**)malloc(listSize * sizeof(char*));
  int parts = splitString(trimmedIPAddress, '.', strList, listSize);

  if (parts != listSize) {
    statusStr = "Invalid IP address format";
    free(strList);
    return false;
  }

  for (int i = 0; i < listSize; i++) {
    int num = atoi(strList[i]);
    if (num < 0 || num > 255) {
      statusStr = "Invalid IP address value";
      for (int j = 0; j < listSize; j++) {
        free(strList[j]);
      }
      free(strList);
      return false;
    }
  }

  pairingData data = {
    .IPv4 = {(uint8_t)atoi(strList[0]), (uint8_t)atoi(strList[1]),
             (uint8_t)atoi(strList[2]), (uint8_t)atoi(strList[3])}
  };
  pairingDataFromComputer = data;

  char buf[60];
  sprintf(buf, "Receive pairing data from Computer:\n %u:%u:%u:%u",
          data.IPv4[0], data.IPv4[1], data.IPv4[2], data.IPv4[3]);
  statusStr = buf;

  for (int i = 0; i < listSize; i++) {
    free(strList[i]);
  }
  free(strList);

  return true;
}

String Pairing::basicInformation() {
  return "[Error] This is Base class";
}

String Pairing::myRole() {
  return String("[Error] This is Base class");
}

// Copied from riberry/firmware/atom_s3_i2c_display/lib/com/communication_base.cpp
// Use char* instead of String for output[] argument
// to minimize dynamic memory usage and reduce the risk of fragmentation.
int splitString(const String &input, char delimiter, char* output[], int maxParts) {
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
