#include <pairing_mode.h>
#include <string_utils.h>

PairingMode::PairingMode()
  : Mode("PairingMode") {
    _pairing_com.setupESPNOW();
}

void PairingMode::task(PrimitiveLCD &lcd, CommunicationBase &com) {
  String currentMessage;
  String lastMessage = "";
  String prevStr = "";

  while (true) {
    com.setRequestStr(getModeName());
#ifdef SENDER
    instance->pairing_com.sendPairingData();
#else
#endif

    if (!compareIgnoringEscapeSequences(prevStr, lcd.color_str)) {
      prevStr = lcd.color_str;
      _pairing_com.receivePairingData(lcd.color_str);
    }

    currentMessage = _pairing_com.basicInformation();
    if (currentMessage != lastMessage) {
      lastMessage = currentMessage;
      lcd.drawBlack();
      lcd.printColorText(currentMessage);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
