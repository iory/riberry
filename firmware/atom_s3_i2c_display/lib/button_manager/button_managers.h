#ifndef BUTTON_MANAGERS_H
#define BUTTON_MANAGERS_H

#include <vector>
#include "button_manager.h"

/**
 * @brief Class to manage multiple ButtonManager instances.
 */
class ButtonManagers {
public:
  /**
   * @brief Constructor to initialize the ButtonManagers with multiple pins.
   *
   * @param pins A vector of pin numbers for each button.
   * @param activeLow Determines whether the buttons are active low. Default is true.
   * @param pullupActive Determines whether the internal pull-up resistor is used. Default is false.
   */
  ButtonManagers(const std::vector<int>& pins, bool activeLow = true, bool pullupActive = false);

  /**
   * @brief Initialize all ButtonManager instances.
   */
  void begin();

  /**
   * @brief Periodically update all ButtonManager instances.
   */
  void tickAll();

  /**
   * @brief Get the current state of a specific button.
   *
   * @param index The index of the button.
   * @return The current ButtonState of the specified button.
   */
  ButtonState getButtonState(size_t index);

  /**
   * @brief Reset the state of a specific button to NOT_CHANGED.
   *
   * @param index The index of the button.
   * @return The updated ButtonState of the specified button.
   */
  ButtonState resetButtonState(size_t index);

  size_t getNumberOfButtons();

  ButtonManager getButtonManager(size_t index);

  void createTask(uint8_t xCoreID);


private:
  std::vector<ButtonManager> buttonManagers; /**< Vector to hold multiple ButtonManager instances. */

  static void task(void *parameter);
};

#endif // BUTTON_MANAGERS_H