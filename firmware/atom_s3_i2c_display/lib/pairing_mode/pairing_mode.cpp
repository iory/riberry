#include <pairing_mode.h>
#include <string_utils.h>

PairingMode* PairingMode::instance = nullptr;

PairingMode::PairingMode(PrimitiveLCD &lcd, CommunicationBase &com)
  : lcd(lcd), comm(com), Mode("PairingMode") {
    instance = this;
    instance->_pairing_com.setupESPNOW();
}

void PairingMode::task(void *parameter) {
  String currentMessage;
  String lastMessage = "";
  String prevStr = "";

  while (true) {
    instance->comm.setRequestStr(instance->getModeName());
#ifdef SENDER
    instance->pairing_com.sendPairingData();
#else
#endif

    if (!compareIgnoringEscapeSequences(prevStr, instance->lcd.color_str)) {
      prevStr = instance->lcd.color_str;
      instance->_pairing_com.receivePairingData(instance->lcd.color_str);
    }

    currentMessage = instance->_pairing_com.basicInformation();
    if (currentMessage != lastMessage) {
      lastMessage = currentMessage;
      instance->lcd.drawBlack();
      instance->lcd.printColorText(currentMessage);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void PairingMode::createTask(uint8_t xCoreID) {
  xTaskCreatePinnedToCore(task, "Pairing Mode", 2048, NULL, 1, &taskHandle, xCoreID);
}
