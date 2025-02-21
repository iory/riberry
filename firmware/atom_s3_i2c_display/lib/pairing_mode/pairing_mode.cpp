#include <color.h>
#include <mac_address.h>
#include <pairing_mode.h>
#include <string_utils.h>

PairingMode::PairingMode(ButtonManager &button_manager, Pairing &pairing, CommunicationBase &com)
    : Mode("PairingMode"), button_manager(button_manager), pairing(pairing), com(com) {}

void PairingMode::task(PrimitiveLCD &lcd, CommunicationBase &com) {
    prevStr = "";
    ButtonState buttonState;
    ButtonState previousButtonState = NOT_CHANGED;
    uint8_t xCoreID = 1;
    preferences.begin("pairing_mode", false);
    preferences.getBytes("role", &com.role, sizeof(Role));

    pairing.stopPairing();
    std::vector<String> pairedMACs = pairing.getPairedMACAddresses();
    unsigned long pairingStartTime = 0;
    while (running) {
        // TODO: Create a function to return an appropriate text size for each
        // display
#ifdef ATOM_S3
        lcd.setTextSize(1.2);
#elif defined(USE_M5STACK_BASIC)
        lcd.setTextSize(2.0);
#endif

        buttonState = button_manager.getButtonState();
        String displayText = "Button State: " + String(buttonState) + "\n";
        if (previousButtonState != buttonState) {
            previousButtonState = buttonState;
            if (buttonState == SINGLE_CLICK) {
                if (!pairing.isPairingActive()) {
                    pairingStartTime = millis();
                    pairing.startPairing();
                } else {
                    pairing.stopPairing();
                }
            } else if (buttonState == DOUBLE_CLICK) {
                if (com.role == Role::Main) {
                    com.role = Role::Secondary;
                    preferences.putBytes("role", &com.role, sizeof(Role));
                } else {
                    com.role = Role::Main;
                    preferences.putBytes("role", &com.role, sizeof(Role));
                }
                pairing.reset();
                pairing.setupBroadcastPeer();
            }
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
        displayText += "2tap Role: ";
        displayText += getRoleStr(com.role);
        displayText += "\n\nMy name:\n" + fancyMacAddress(pairing.getMyMACAddress().c_str()) + "\n";
        if (!pairedMACs.empty()) {
            displayText += "\nPaired devices:\n";
            for (const auto &mac : pairedMACs) {
                displayText += fancyMacAddress(mac.c_str()) + "\n";
            }
        }
        if (!lcd.color_str.isEmpty()) {
            displayText += String("\n") + "MST" + lcd.color_str;
        }
        displayText += String("\n") + pairing.getStatus();

        if (!compareIgnoringEscapeSequences(prevStr, displayText)) {
            prevStr = displayText;
            lcd.drawBlack();
            lcd.printColorText(displayText);
        }
        // short delay to catch button presses
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void PairingMode::deleteTask() {
    pairing.deleteTask();
    Mode::deleteTask();
    com.stopPairing();
    WiFi.disconnect(true);
}

BaseType_t PairingMode::createTask(uint8_t xCoreID, PrimitiveLCD &lcd, CommunicationBase &com) {
    WiFi.reconnect();
    pairing.createTask(1);
    BaseType_t taskCreated = Mode::createTask(1, lcd, com);
    com.startPairing();
    return taskCreated;
}
