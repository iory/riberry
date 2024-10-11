#include <atom_s3_button.h>

AtomS3Button::AtomS3Button(int pin, bool activeLow, bool pullupActive)
  : btn(pin, activeLow, pullupActive), currentButtonState(RESET), clicked(false), doubleClicked(false), longPressed(false) {
  begin();
  createTask(0);
}

void AtomS3Button::begin() {
  btn.setClickMs(200); // Timeout used to distinguish single clicks from double clicks. (msec)
  btn.attachClick(handleClickStatic, this);
  btn.attachDoubleClick(handleDoubleClickStatic, this);
  btn.attachMultiClick(handleMultiClickStatic, this);
  btn.attachLongPressStart(handleLongPressStatic, this);
  btn.attachLongPressStop(handleLongPressEndStatic, this);
}

void AtomS3Button::tick() {
  btn.tick();
}

ButtonState AtomS3Button::getButtonState() {
  return currentButtonState;
}

ButtonState AtomS3Button::notChangedButtonState() {
  return currentButtonState = NOT_CHANGED;
}

// Static handlers (call each member function)
void AtomS3Button::handleClickStatic(void *instance) {
  static_cast<AtomS3Button*>(instance)->handleClick();
}

void AtomS3Button::handleDoubleClickStatic(void *instance) {
  static_cast<AtomS3Button*>(instance)->handleDoubleClick();
}

void AtomS3Button::handleMultiClickStatic(void *instance) {
  static_cast<AtomS3Button*>(instance)->handleMultiClick();
}

void AtomS3Button::handleLongPressStatic(void *instance) {
  static_cast<AtomS3Button*>(instance)->handleLongPress();
}

void AtomS3Button::handleLongPressEndStatic(void *instance) {
  static_cast<AtomS3Button*>(instance)->handleLongPressEnd();
}

// Non-static member functions perform the actual processing
void AtomS3Button::handleClick() {
  currentButtonState = SINGLE_CLICK;
  clicked = true;
}

void AtomS3Button::handleDoubleClick() {
  currentButtonState = DOUBLE_CLICK;
  doubleClicked = true;
}

void AtomS3Button::handleMultiClick() {
  int n = btn.getNumberClicks();
  switch (n) {
  case 1: currentButtonState = SINGLE_CLICK; break;
  case 2: currentButtonState = DOUBLE_CLICK; break;
  case 3: currentButtonState = TRIPLE_CLICK; break;
  case 4: currentButtonState = QUADRUPLE_CLICK; break;
  case 5: currentButtonState = QUINTUPLE_CLICK; break;
  case 6: currentButtonState = SEXTUPLE_CLICK; break;
  case 7: currentButtonState = SEPTUPLE_CLICK; break;
  case 8: currentButtonState = OCTUPLE_CLICK; break;
  case 9: currentButtonState = NONUPLE_CLICK; break;
  case 10: currentButtonState = DECUPLE_CLICK; break;
  default: currentButtonState = DECUPLE_CLICK; // Handle case where n > 10
  }
}

void AtomS3Button::handleLongPress() {
  currentButtonState = PRESSED;
  longPressed = true;
}

void AtomS3Button::handleLongPressEnd() {
  currentButtonState = RELEASED;
}

//Referencing M5's wasPressed()
bool AtomS3Button::wasClicked() {
  if (clicked) {
    clicked = false;
    return true;
  }
  else
    return false;
}

bool AtomS3Button::wasDoubleClicked() {
  if (doubleClicked) {
    doubleClicked = false;
    return true;
  }
  else
    return false;
}

bool AtomS3Button::wasLongPressed() {
  if (longPressed) {
    longPressed = false;
    return true;
  }
  else
    return false;
}

void AtomS3Button::task(void *parameter) {
  AtomS3Button *buttonInstance = static_cast<AtomS3Button *>(parameter);
  while (true) {
    buttonInstance->tick();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void AtomS3Button::createTask(uint8_t xCoreID) {
  xTaskCreatePinnedToCore(task, "Button Task", 2048, this, 24, NULL, xCoreID);
}
