#include <color.h>
#include <mac_address.h>
#include <pairing_mode.h>
#include <string_utils.h>

PairingMode::PairingMode(ButtonManager &button_manager, Pairing &pairing)
    : Mode("PairingMode"), button_manager(button_manager), pairing(pairing) {}

void PairingMode::task(PrimitiveLCD &lcd, CommunicationBase &com) {
    prevStr = "";
    ButtonState buttonState;
    uint8_t xCoreID = 0;
    preferences.begin("pairing_mode", false);
    preferences.getBytes("role", &currentRole, sizeof(Role));

    pairing.stopPairing();
    pairing.startBackgroundTask(xCoreID);
    std::vector<String> pairedMACs = pairing.getPairedMACAddresses();
    unsigned long pairingStartTime = 0;
    while (true) {
        // TODO: Create a function to return an appropriate text size for each
        // display
#ifdef ATOM_S3
        lcd.setTextSize(1.2);
#elif defined(USE_M5STACK_BASIC)
        lcd.setTextSize(2.0);
#endif

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
        } else if (buttonState == DOUBLE_CLICK) {
            if (currentRole == Role::Main) {
                currentRole = Role::Secondary;
                preferences.putBytes("role", &currentRole, sizeof(Role));
            } else {
                currentRole = Role::Main;
                preferences.putBytes("role", &currentRole, sizeof(Role));
            }
            pairing.reset();
        }

        pairedMACs = pairing.getPairedMACAddresses();

        if (pairing.isPairingActive() && millis() - pairingStartTime >= 3000) {
            pairing.stopPairing();
        }

        // Display
        displayText += "1tap ";
        if (pairing.isPairingActive()) {
            displayText += Color::Foreground::BLACK + Color::Background::GREEN + "Pairing ON\n" +
                           Color::Background::RESET + Color::Foreground::RESET;
        } else {
            displayText += Color::Background::RED + "Pairing OFF\n" + Color::Background::RESET;
        }
        displayText += "2tap ";
        if (currentRole == Role::Main) {
            displayText += "Role: Main\n";
        } else {
            displayText += "Role: Second\n";
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
        // short delay to catch button presses
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void PairingMode::suspendTask() {
    pairing.stopPairing();
    Mode::suspendTask();
}

void PairingMode::resumeTask() {
    unsigned long pairingStartTime = 0;
    Mode::resumeTask();
}
