#ifndef MODE_H
#define MODE_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Arduino.h> // To use String class

#include <primitive_lcd.h>
#include <communication_base.h>

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

  bool isTaskSuspended() const {
    return eTaskGetState(taskHandle) == eSuspended;
  }

  bool isTaskCreated() const {
    return taskHandle != NULL;
  }

  void waitForTaskSuspended() const {
    while (!isTaskSuspended()) {
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }

  virtual void task(PrimitiveLCD& lcd, CommunicationBase& com) = 0;

  static void startTaskImpl(void* _params) {
    auto* params = static_cast<std::tuple<Mode*, PrimitiveLCD*, CommunicationBase*>*>(_params);
    Mode* _this = std::get<0>(*params);
    PrimitiveLCD* lcd = std::get<1>(*params);
    CommunicationBase* com = std::get<2>(*params);
    _this->task(*lcd, *com);
    delete params;
  }

  void createTask(uint8_t xCoreID, PrimitiveLCD& lcd, CommunicationBase& com) {
    auto* params = new std::tuple<Mode*, PrimitiveLCD*, CommunicationBase*>(this, &lcd, &com);
    xTaskCreatePinnedToCore(this->startTaskImpl, getModeName().c_str(), 2048, params, 1, &taskHandle, xCoreID);
  }

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
