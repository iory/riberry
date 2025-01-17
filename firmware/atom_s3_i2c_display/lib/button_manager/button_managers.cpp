#include "button_managers.h"

/**
 * @brief Constructor to initialize the ButtonManagers with multiple pins.
 */
ButtonManagers::ButtonManagers(const std::vector<int>& pins, bool activeLow, bool pullupActive) {
  for (int pin : pins) {
    buttonManagers.emplace_back(pin, activeLow, pullupActive);
  }
}

/**
 * @brief Initialize all ButtonManager instances.
 */
void ButtonManagers::begin() {
  for (auto& manager : buttonManagers) {
    manager.begin();
  }
}

/**
 * @brief Periodically update all ButtonManager instances.
 */
void ButtonManagers::tickAll() {
  for (auto& manager : buttonManagers) {
    manager.tick();
  }
}

/**
 * @brief Get the current state of a specific button.
 *
 * @param index The index of the button.
 * @return The current ButtonState of the specified button.
 */
ButtonState ButtonManagers::getButtonState(size_t index) {
  if (index < buttonManagers.size()) {
    return buttonManagers[index].getButtonState();
  }
  return NOT_CHANGED; // Return a default state if index is out of bounds
}

/**
 * @brief Reset the state of a specific button to NOT_CHANGED.
 *
 * @param index The index of the button.
 * @return The updated ButtonState of the specified button.
 */
ButtonState ButtonManagers::resetButtonState(size_t index) {
  if (index < buttonManagers.size()) {
    return buttonManagers[index].notChangedButtonState();
  }
  return NOT_CHANGED; // Return a default state if index is out of bounds
}

size_t ButtonManagers::getNumberOfButtons() {
  return buttonManagers.size();
}

ButtonManager ButtonManagers::getButtonManager(size_t index) {
  index = min(buttonManagers.size(), max(static_cast<size_t>(0), index));
  return buttonManagers[index];
}

void ButtonManagers::task(void *parameter) {
  ButtonManagers *buttonInstance = static_cast<ButtonManagers *>(parameter);
  while (true) {
    buttonInstance->tickAll();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void ButtonManagers::createTask(uint8_t xCoreID) {
  xTaskCreatePinnedToCore(task, "Button Task", 2048, this, 24, NULL, xCoreID);
}
