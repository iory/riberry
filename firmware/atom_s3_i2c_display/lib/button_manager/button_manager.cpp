#include <button_manager.h>

ButtonManager::ButtonManager(int pin, bool activeLow, bool pullupActive)
    : btn(pin, activeLow, pullupActive),
      currentButtonState(RESET),
      clicked(false),
      doubleClicked(false),
      longPressed(false),
      longPressedState(false),
      ExecutionTimer("ButtonManager") {
    begin();
}

void ButtonManager::begin() {
    btn.setClickMs(200);  // Timeout used to distinguish single clicks from
                          // double clicks. (msec)
    btn.attachClick(handleClickStatic, this);
    btn.attachDoubleClick(handleDoubleClickStatic, this);
    btn.attachMultiClick(handleMultiClickStatic, this);
    btn.attachLongPressStart(handleLongPressStatic, this);
    btn.attachLongPressStop(handleLongPressEndStatic, this);
}

void ButtonManager::tick() { btn.tick(); }

ButtonState ButtonManager::getButtonState() const { return currentButtonState; }

ButtonState ButtonManager::notChangedButtonState() { return currentButtonState = NOT_CHANGED; }

// Static handlers (call each member function)
void ButtonManager::handleClickStatic(void *instance) {
    static_cast<ButtonManager *>(instance)->handleClick();
}

void ButtonManager::handleDoubleClickStatic(void *instance) {
    static_cast<ButtonManager *>(instance)->handleDoubleClick();
}

void ButtonManager::handleMultiClickStatic(void *instance) {
    static_cast<ButtonManager *>(instance)->handleMultiClick();
}

void ButtonManager::handleLongPressStatic(void *instance) {
    static_cast<ButtonManager *>(instance)->handleLongPress();
}

void ButtonManager::handleLongPressEndStatic(void *instance) {
    static_cast<ButtonManager *>(instance)->handleLongPressEnd();
}

// Non-static member functions perform the actual processing
void ButtonManager::handleClick() {
    currentButtonState = SINGLE_CLICK;
    clicked = true;
}

void ButtonManager::handleDoubleClick() {
    currentButtonState = DOUBLE_CLICK;
    doubleClicked = true;
}

void ButtonManager::handleMultiClick() {
    int n = btn.getNumberClicks();
    switch (n) {
        case 1:
            currentButtonState = SINGLE_CLICK;
            break;
        case 2:
            currentButtonState = DOUBLE_CLICK;
            break;
        case 3:
            currentButtonState = TRIPLE_CLICK;
            break;
        case 4:
            currentButtonState = QUADRUPLE_CLICK;
            break;
        case 5:
            currentButtonState = QUINTUPLE_CLICK;
            break;
        case 6:
            currentButtonState = SEXTUPLE_CLICK;
            break;
        case 7:
            currentButtonState = SEPTUPLE_CLICK;
            break;
        case 8:
            currentButtonState = OCTUPLE_CLICK;
            break;
        case 9:
            currentButtonState = NONUPLE_CLICK;
            break;
        case 10:
            currentButtonState = DECUPLE_CLICK;
            break;
        default:
            currentButtonState = DECUPLE_CLICK;  // Handle case where n > 10
    }
}

void ButtonManager::handleLongPress() {
    currentButtonState = PRESSED;
    longPressed = true;
    longPressedState = true;
}

void ButtonManager::handleLongPressEnd() {
    currentButtonState = RELEASED;
    longPressedState = false;
}

// Referencing M5's wasPressed()
bool ButtonManager::wasClicked() {
    if (clicked) {
        clicked = false;
        return true;
    } else
        return false;
}

bool ButtonManager::wasDoubleClicked() {
    if (doubleClicked) {
        doubleClicked = false;
        return true;
    } else
        return false;
}

bool ButtonManager::wasLongPressed() {
    if (longPressed) {
        longPressed = false;
        return true;
    } else
        return false;
}

unsigned long ButtonManager::getPressedMs() {
    if (longPressedState) {
        return btn.getPressedMs();
    } else {
        return 0;
    }
}

void ButtonManager::task(void *parameter) {
    ButtonManager *buttonInstance = static_cast<ButtonManager *>(parameter);
    ButtonState lastState = buttonInstance->getButtonState();
    unsigned long lastChangeTime = millis();
    while (true) {
        buttonInstance->tick();
        ButtonState currentState = buttonInstance->getButtonState();

        if (currentState != lastState) {
            lastState = currentState;
            lastChangeTime = millis();
        } else if (millis() - lastChangeTime >= 1000) {
            buttonInstance->notChangedButtonState();
        }

        buttonInstance->delayWithTimeTracking(pdMS_TO_TICKS(10));
    }
}

void ButtonManager::createTask(uint8_t xCoreID) {
    this->xCoreID = xCoreID;
    xTaskCreatePinnedToCore(task, "Button Task", 2048, this, 24, NULL, xCoreID);
}
