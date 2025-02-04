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

    pairing.startBackgroundTask(1);
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
        pairing.startBackgroundTask(0);
        pairingStatus = "pairing\n";
    } else {
        pairing.deleteTask();
        esp_now_deinit();
        WiFi.disconnect(true);
        pairingStatus = "not pairing\n";
    }
    std::vector<String> pairedMAC = pairing.getPairedMACAddresses();
    for (const auto &mac : pairedMAC) {
        pairingStatus += String(main_or_secondary) + "\nPaired MAC:\n" + mac;
    }
    printToLCD(pairingStatus);
    delay(100);
}