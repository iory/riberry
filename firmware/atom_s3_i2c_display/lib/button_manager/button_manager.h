#ifndef BUTTON_MANAGER_H
#define BUTTON_MANAGER_H

#include <OneButton.h>

#include "execution_timer.h"

#ifdef ATOM_S3
constexpr byte BUTTON_PIN = 41;
#elif defined(USE_M5STACK_BASIC)
constexpr byte BUTTON_PIN = 39;
#else
constexpr byte BUTTON_PIN = 41;
#endif

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

class ButtonManager : public ExecutionTimer {
public:
    /**
     * @brief Constructor to initialize the button with specified settings.
     *
     * @param pin The pin number the button is connected to. Default is 41.
     * @param activeLow Determines whether the button is active low. Default is
     * true.
     * @param pullupActive Determines whether the internal pull-up resistor is
     * used. Default is false.
     */
    ButtonManager(int pin = BUTTON_PIN, bool activeLow = true, bool pullupActive = false);

    void begin();

    /**
     * @brief Update the button state, should be called periodically.
     */
    void tick();

    ButtonState getButtonState() const;

    /**
     * @brief Reset the button state to NOT_CHANGED.
     *
     * @return The updated ButtonState.
     */
    ButtonState notChangedButtonState();

    bool wasClicked();
    bool wasDoubleClicked();
    bool wasLongPressed();
    unsigned long getPressedMs();

    void createTask(uint8_t xCoreID);

private:
    OneButton btn;                  /**< Instance of the OneButton library to manage the button. */
    ButtonState currentButtonState; /**< Current state of the button. */
    bool clicked;
    bool doubleClicked;
    bool longPressed;
    bool longPressedState;

    static void handleClickStatic(void *instance);
    static void handleDoubleClickStatic(void *instance);
    static void handleMultiClickStatic(void *instance);
    static void handleLongPressStatic(void *instance);
    static void handleLongPressEndStatic(void *instance);
    void handleClick();
    void handleDoubleClick();
    void handleMultiClick();
    void handleLongPress();
    void handleLongPressEnd();

    static void task(void *parameter);
};

#endif
