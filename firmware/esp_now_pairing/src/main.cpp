#define LGFX_M5ATOMS3
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>
static LGFX lcd;

#include <pairing_sender.h>
#include <pairing_receiver.h>
#ifdef SENDER
PairingSender com;
#endif
#ifdef RECEIVER
PairingReceiver com;
#endif

String lastMessage = "";

void print(const String &Message) {
  lcd.fillScreen(lcd.color565(0, 0, 0));  // Fill the screen with black
  lcd.setCursor(0, 0);
  lcd.println(Message);
}

void init_lcd () {
  lcd.init();
  lcd.setRotation(0);
  lcd.clear();
  lcd.setTextSize(1.2);
}

// Setup Serial with host computer
void start_serial () {
  USBSerial.begin(115200);
  // If this delay is short, ESP32 may become core panic
  // Delay to stabilize serial connection
  delay(1500);
  USBSerial.println(com.myRole());
  // Receive pairing data (host computer data)
  delay(100);  // Wait for pairing data from computer
  com.receivePairingDataFromComputer();
}

void setup() {
  init_lcd();
  print(com.basicInformation());

  start_serial();
  print(com.basicInformation());

  // WiFi setup must be done in setup() function. Not in global scope.
  com.setupESPNOW();
  print(com.basicInformation());
}

void loop() {
  com.impl();

  String currentMessage = com.basicInformation();
  if (currentMessage != lastMessage) {
    print(currentMessage);
    lastMessage = currentMessage;
  }

  delay(100);
}
