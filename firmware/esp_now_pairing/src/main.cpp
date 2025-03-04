#define LGFX_M5ATOMS3
#define LGFX_USE_V1
#include <LGFX_AUTODETECT.hpp>
#include <LovyanGFX.hpp>

#include "pairing.h"

#ifdef ENV_MAIN
const char *main_or_secondary = "Main";
#else
const char *main_or_secondary = "Secondary";
#endif

static LGFX lcd;
Pairing pairing;
String previousMessage = "";

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
    delay(1500);
}

PairingData dataToSend = {{255, 255, 255, 255}};

void loop() {
    if (USBSerial.available() > 0) {
        uint8_t header = USBSerial.read();
        switch (header) {
            case 0x11:
                USBSerial.println(main_or_secondary);
                break;
            case 0x12: {
                std::map<String, PairingData> pairedDataMap = pairing.getPairedData();
                if (!pairedDataMap.empty()) {
                    auto it = pairedDataMap.begin();
                    USBSerial.write(it->second.IPv4[0]);
                    USBSerial.write(it->second.IPv4[1]);
                    USBSerial.write(it->second.IPv4[2]);
                    USBSerial.write(it->second.IPv4[3]);
                }
                break;
            }
            case 0x13:
                for (int i = 0; i < 4; i++) {
                    dataToSend.IPv4[i] = USBSerial.read();
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
