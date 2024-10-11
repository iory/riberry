#ifndef ATOM_S3_BUTTON_H
#define ATOM_S3_BUTTON_H

#include <OneButton.h>

/**
 * @brief Button click states enumeration.
 */
enum ButtonState {
  NOT_CHANGED,
  SINGLE_CLICK,
  DOUBLE_CLICK,
  TRIPLE_CLICK,
  QUADRUPLE_CLICK,  // 4 clicks
  QUINTUPLE_CLICK,  // 5 clicks
  SEXTUPLE_CLICK,   // 6 clicks
  SEPTUPLE_CLICK,   // 7 clicks
  OCTUPLE_CLICK,    // 8 clicks
  NONUPLE_CLICK,    // 9 clicks
  DECUPLE_CLICK,    // 10 clicks
  PRESSED,
  RELEASED,
  RESET,
  BUTTON_STATE_COUNT
};

/**
 * @brief Class to handle a button on the AtomS3 device using the OneButton library.
 */
class AtomS3Button {
public:
  /**
   * @brief Constructor to initialize the button with specified settings.
   *
   * @param pin The pin number the button is connected to. Default is 41.
   * @param activeLow Determines whether the button is active low. Default is true.
   * @param pullupActive Determines whether the internal pull-up resistor is used. Default is false.
   */
  AtomS3Button(int pin = 41, bool activeLow = true, bool pullupActive = false);

  /**
   * @brief Initialize the button with event handlers.
   */
  void begin();

  /**
   * @brief Update the button state, should be called periodically.
   */
  void tick();

  /**
   * @brief Get the current state of the button.
   *
   * @return The current ButtonState.
   */
  ButtonState getButtonState();

  /**
   * @brief Reset the button state to NOT_CHANGED.
   *
   * @return The updated ButtonState.
   */
  ButtonState notChangedButtonState();

  /**
   * @brief Non-static method to check single click occured.
   */
  bool wasClicked();

  /**
   * @brief Non-static method to check double click occured.
   */
  bool wasDoubleClicked();

  /**
   * @brief Non-static method to check long press occured.
   */
  bool wasLongPressed();

  /**
   * @brief Create a FreeRTOS task for handling button events.
   *
   * @param xCoreID The core on which to run the task (CPU core ID).
   */
  void createTask(uint8_t xCoreID);

private:
  OneButton btn;  /**< Instance of the OneButton library to manage the button. */
  ButtonState currentButtonState; /**< Current state of the button. */
  bool clicked;
  bool doubleClicked;
  bool longPressed;

  /**
   * @brief Static method to handle single-click events.
   *
   * @param instance A pointer to the AtomS3Button instance.
   */
  static void handleClickStatic(void *instance);

  /**
   * @brief Static method to handle double-click events.
   *
   * @param instance A pointer to the AtomS3Button instance.
   */
  static void handleDoubleClickStatic(void *instance);

  /**
   * @brief Static method to handle multi-click events.
   *
   * @param instance A pointer to the AtomS3Button instance.
   */
  static void handleMultiClickStatic(void *instance);

  /**
   * @brief Static method to handle long-press events.
   *
   * @param instance A pointer to the AtomS3Button instance.
   */
  static void handleLongPressStatic(void *instance);

  /**
   * @brief Static method to handle long-press-end events.
   *
   * @param instance A pointer to the AtomS3Button instance.
   */
  static void handleLongPressEndStatic(void *instance);

  /**
   * @brief Non-static method to handle single-click events.
   */
  void handleClick();

  /**
   * @brief Non-static method to handle double-click events.
   */
  void handleDoubleClick();

  /**
   * @brief Non-static method to handle multi-click events.
   */
  void handleMultiClick();

  /**
   * @brief Non-static method to handle long-press events.
   */
  void handleLongPress();

  /**
   * @brief Non-static method to handle long-press-end events.
   */
  void handleLongPressEnd();

  /**
   * @brief The task function for FreeRTOS to continuously monitor button state.
   *
   * @param parameter A pointer to the AtomS3Button instance.
   */
  static void task(void *parameter);
};

#endif
