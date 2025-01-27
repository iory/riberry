#define LGFX_M5ATOMS3
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>
static LGFX lcd;

#ifdef SENDER
#include <pairing_sender.h>
PairingSender com;
#endif
#ifdef RECEIVER
#include <pairing_receiver.h>
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

  String receivedData;
  while (USBSerial.available() <= 0) {
      delay(10);
  };
  while (1) {
    receivedData = USBSerial.readStringUntil('\n');
    if (com.receivePairingData(receivedData)) {
      break;
    }
  }
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
  com.impl(USBSerial);

  String currentMessage = com.basicInformation();
  if (currentMessage != lastMessage) {
    print(currentMessage);
    lastMessage = currentMessage;
  }

  delay(100);
}
