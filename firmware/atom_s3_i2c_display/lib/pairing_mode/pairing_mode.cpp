#include <color.h>
#include <pairing_mode.h>
#include <string_utils.h>

#include <mac_address.h>

PairingMode::PairingMode(ButtonManager &button_manager, Pairing &pairing)
    : Mode("PairingMode"), button_manager(button_manager), pairing(pairing) {}

void PairingMode::task(PrimitiveLCD &lcd, CommunicationBase &com) {
  prevStr = "";
  ButtonState buttonState;
  uint8_t xCoreID = 0;

  pairing.stopPairing();
  pairing.startBackgroundTask(xCoreID);
  std::vector<String> pairedMACs = pairing.getPairedMACAddresses();
  unsigned long pairingStartTime = 0;
  while (true) {
    // TODO: Create a function to return an appropriate text size for each display
#ifdef ATOM_S3
    lcd.setTextSize(1.2);
#elif defined(USE_M5STACK_BASIC)
    lcd.setTextSize(2.0);
#endif

    com.setRequestStr(getModeName());

    buttonState = button_manager.getButtonState();
    button_manager.notChangedButtonState();
    String displayText = "";
    if (buttonState == SINGLE_CLICK) {
      if (!pairing.isPairingActive()) {
        pairingStartTime = millis();
        pairing.startPairing();
      } else {
        pairing.stopPairing();
      }
    }
    else if (buttonState == DOUBLE_CLICK) {
      pairing.reset();
    }
    pairedMACs = pairing.getPairedMACAddresses();

    if (pairing.isPairingActive() && millis() - pairingStartTime >= 3000) {
      pairing.stopPairing();
    }
    if (pairing.isPairingActive()) {
      displayText += Color::Foreground::BLACK + Color::Background::GREEN +
                     "Pairing active\n" + Color::Background::RESET +
                     Color::Foreground::RESET;
    } else {
      displayText += Color::Background::RED + "Pairing inactive\n" +
                     Color::Background::RESET;
    }
    displayText += "\nMy name:\n" + fancyMacAddress(pairing.getMyMACAddress().c_str()) + "\n";
    if (!pairedMACs.empty()) {
      displayText += "\nPaired devices:\n";
      for (const auto &mac : pairedMACs) {
        displayText += fancyMacAddress(mac.c_str()) + "\n";
      }
    }

    if (!compareIgnoringEscapeSequences(prevStr, displayText)) {
      prevStr = displayText;
      lcd.drawBlack();
      lcd.printColorText(displayText);
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void PairingMode::suspendTask() {
  pairing.stopPairing();
  Mode::suspendTask();
}

void PairingMode::resumeTask() {
  unsigned long pairingStartTime = 0;
  prevStr = "";
  Mode::resumeTask();
}
