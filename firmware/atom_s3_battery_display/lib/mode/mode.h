#ifndef MODE_H
#define MODE_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Arduino.h> // To use String class

/**
 * @brief Base class for handling FreeRTOS tasks.
 */
class Mode {
public:
  Mode(const String& name)  : taskHandle(NULL), modeName(name) {}

  /**
   * @brief Suspend the task.
   */
  void suspendTask() {
    if (taskHandle != NULL) {
      vTaskSuspend(taskHandle);
    }
  }

  /**
   * @brief Resume the task.
   */
  void resumeTask() {
    if (taskHandle != NULL) {
      vTaskResume(taskHandle);
    }
  }

  bool isTaskSuspended() {
    return eTaskGetState(taskHandle) == eSuspended;
  }

  void waitForTaskSuspended() {
    while (!isTaskSuspended()) {
      delay(100);
    }
  }

  /**
   * @brief Create the task. To be implemented in derived classes.
   *
   * @param xCoreID The core to pin the task to.
   */
  virtual void createTask(uint8_t xCoreID) = 0;

  /**
   * @brief Get the mode name.
   * @return The name of the mode.
   */
  String getModeName() const {
    return modeName;
  }

protected:
  TaskHandle_t taskHandle;
  String modeName;
};

#endif // MODE_H
