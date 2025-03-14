#define LGFX_M5ATOMS3
#define LGFX_USE_V1
#include <SerialTransfer.h>

#include <LGFX_AUTODETECT.hpp>
#include <LovyanGFX.hpp>

#include "pairing.h"

#ifdef ENV_MAIN
String main_or_secondary = "Main";
#else
String main_or_secondary = "Secondary";
#endif

static LGFX lcd;
Pairing pairing;
String previousMessage = "";
SerialTransfer transfer;
uint8_t buffer[256];

template <typename T>
void write(const T &data, const size_t len) {
    transfer.txObj(data, 0, len);
    transfer.sendData(len);
}

void printToLCD(const String &message) {
    if (message == previousMessage) {
        return;
    }
    previousMessage = message;
    lcd.fillScreen(lcd.color565(0, 0, 0));
    lcd.setCursor(0, 0);
    lcd.println(message);
}

void initLCD() {
    lcd.init();
    lcd.setRotation(0);
    lcd.clear();
    lcd.setTextSize(1.2);
}

void setup() {
    initLCD();
    printToLCD("Initializing...");

    int xCoreID = 1;
    pairing.createTask(xCoreID);
    printToLCD(String(main_or_secondary) + "\nMy MAC:\n" + pairing.getMyMACAddress());

    USBSerial.begin(115200);
    transfer.begin(USBSerial);
    delay(1500);
    printToLCD(String(main_or_secondary) + "\nMy MAC:\n" + pairing.getMyMACAddress() + "\nReady");
}

PairingData dataToSend = {{255, 255, 255, 255}};

void loop() {
    size_t available = transfer.available();
    if (available > 0) {
        transfer.rxObj(buffer, 0, available);
        switch (buffer[0]) {
            case 0x11:
                write(main_or_secondary, main_or_secondary.length());
                break;
            case 0x12: {
                std::map<String, PairingData> pairedDataMap = pairing.getPairedData();
                if (!pairedDataMap.empty()) {
                    auto it = pairedDataMap.begin();
                    write(it->second.IPv4, 4);
                }
                break;
            }
            case 0x13:
                for (int i = 0; i < 4; i++) {
                    dataToSend.IPv4[i] = buffer[i + 1];
                }
                pairing.setDataToSend(dataToSend);
                break;
            default:
                break;
        }
    }

    std::map<String, PairingData> pairedDataMap = pairing.getPairedData();
    for (const auto &pair : pairedDataMap) {
        String displayMessage = String(main_or_secondary) + "\nMAC:\n" + pair.first + "\nData:\n" +
                                String(pair.second.IPv4[0]) + "." + String(pair.second.IPv4[1]) +
                                "." + String(pair.second.IPv4[2]) + "." +
                                String(pair.second.IPv4[3]);
        printToLCD(displayMessage);
    }
    delay(100);
}
