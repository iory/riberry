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

constexpr byte BUTTON_PIN = 41;

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

    pairing.createTask(1);
    printToLCD(String(main_or_secondary) + "\nMy MAC:\n" + pairing.getMyMACAddress());

    USBSerial.begin(115200);
    delay(1500);
}

PairingData dataToSend = {{255, 255, 255, 255}};
bool pairingActive = false;
bool buttonReleased = true;

void loop() {
    String pairingStatus;
    if (buttonReleased && digitalRead(BUTTON_PIN) == LOW) {
        pairingActive = !pairingActive;
        buttonReleased = false;
    } else if (digitalRead(BUTTON_PIN) == HIGH) {
        buttonReleased = true;
    }
    if (pairingActive) {
        pairing.createTask(1);
        pairingStatus = "pairing\n";
    } else {
        pairing.stopBackgroundTask();
        esp_now_deinit();
        WiFi.disconnect(true);
        pairingStatus = "not pairing\n";
    }

    std::map<String, PairingData> pairedDataMap = pairing.getPairedData();
    printToLCD(pairingStatus);
    for (const auto &pair : pairedDataMap) {
        String displayMessage = pairingStatus + String(main_or_secondary) + "\nMAC:\n" +
                                pair.first + "\nData:\n" + String(pair.second.IPv4[0]) + "." +
                                String(pair.second.IPv4[1]) + "." + String(pair.second.IPv4[2]) +
                                "." + String(pair.second.IPv4[3]);
        printToLCD(displayMessage);
    }
    delay(100);
}