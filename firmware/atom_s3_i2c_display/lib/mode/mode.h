#ifndef MODE_H
#define MODE_H

#include <Arduino.h>  // To use String class
#include <communication_base.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <primitive_lcd.h>
#include <string_utils.h>

/**
 * @brief Base class for handling FreeRTOS tasks.
 */
class Mode {
public:
    Mode(const String& name) : taskHandle(NULL), modeName(name) {}

    /**
     * @brief Suspend the task.
     */
    virtual void suspendTask() {
        if (taskHandle != NULL) {
            vTaskSuspend(taskHandle);
        }
    }

    /**
     * @brief Resume the task.
     */
    virtual void resumeTask() {
        prevStr = "";
        if (taskHandle != NULL) {
            vTaskResume(taskHandle);
        }
    }

    bool isTaskSuspended() const { return eTaskGetState(taskHandle) == eSuspended; }

    bool isTaskCreated() const { return taskHandle != NULL; }

    void waitForTaskSuspended() const {
        while (!isTaskSuspended()) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    virtual void task(PrimitiveLCD& lcd, CommunicationBase& com) {
        while (true) {
            if (handleTimeout(lcd, com) || handleEmptyDisplay(lcd)) continue;

            // The reason for using this here is that the equals function
            // relies on strcmp, which cannot properly handle escape
            // sequences. As a result, it may fail to compare strings
            // correctly.
            if (compareIgnoringEscapeSequences(prevStr, lcd.color_str)) {
                vTaskDelay(pdMS_TO_TICKS(10));
            } else {
                lcd.drawBlack();
                prevStr = lcd.color_str;
                lcd.printColorText(lcd.color_str);
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

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
        xTaskCreatePinnedToCore(this->startTaskImpl, getModeName().c_str(), 2048, params, 1,
                                &taskHandle, xCoreID);
    }

    /**
     * @brief Get the mode name.
     * @return The name of the mode.
     */
    String getModeName() const { return modeName; }

    bool handleTimeout(PrimitiveLCD& lcd, CommunicationBase& com) {
        if (com.checkTimeout()) {
            lcd.drawNoDataReceived();
            lcd.printColorText(getModeName() + "\n");
            vTaskDelay(pdMS_TO_TICKS(500));
            prevStr = "";
            return true;
        } else {
            // Short delay to avoiding watch-dog timer reset
            vTaskDelay(pdMS_TO_TICKS(1 / portTICK_PERIOD_MS));
        }
        return false;
    }

    bool handleEmptyDisplay(PrimitiveLCD& lcd) {
        if (lcd.color_str.isEmpty()) {
            String waitStr = "Waiting for " + getModeName();
            lcd.drawBlack();
            lcd.printColorText(waitStr);
            vTaskDelay(pdMS_TO_TICKS(500));
            return true;
        }
        return false;
    }

protected:
    TaskHandle_t taskHandle;
    String modeName;
    String prevStr;
};

#endif  // MODE_H
