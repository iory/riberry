#ifndef MODE_H
#define MODE_H

#include <Arduino.h>  // To use String class
#include <communication_base.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <mode_type.h>
#include <primitive_lcd.h>
#include <string_utils.h>

#include "execution_timer.h"

class Mode : public ExecutionTimer {
public:
    Mode(const ModeType::Name& name, bool isSkippable = false)
        : ExecutionTimer(ModeType::toString(name)),
          name(name),
          running(false),
          _isSkippable(isSkippable) {}

    virtual void suspendTask() {
        if (taskHandle != NULL) {
            vTaskSuspend(taskHandle);
        }
    }

    virtual void resumeTask(uint8_t xCoreID, PrimitiveLCD& lcd, CommunicationBase& com) {
        prevStr = "";
        if (taskHandle == NULL) {
            createTask(xCoreID, lcd, com);
        } else if (taskHandle != NULL) {
            vTaskResume(taskHandle);
        }
    }

    virtual void deleteTask() {
        prevStr = "";
        stopTaskAndWaitForCompletion();
    }

    bool isTaskSuspended() const { return eTaskGetState(taskHandle) == eSuspended; }

    bool isTaskCreated() const { return taskHandle != NULL; }

    void waitForTaskSuspended() {
        while (!isTaskSuspended()) {
            delayWithTimeTracking(pdMS_TO_TICKS(100));
        }
    }

    void stopTaskAndWaitForCompletion() {
        if (taskHandle != NULL) {
            running = false;
            while (eTaskGetState(taskHandle) != eDeleted) {
                delayWithTimeTracking(pdMS_TO_TICKS(50));
            }
            taskHandle = NULL;
            this->xCoreID = -1;
        }
    }

    // Returns remaining stack size in [byte]
    UBaseType_t remainingStackSize() const {
        if (taskHandle != NULL) {
            return uxTaskGetStackHighWaterMark(taskHandle);
        } else {
            return 0;  // Do not use this function when taskHandle == NULL
        }
    }

    virtual void task(PrimitiveLCD& lcd, CommunicationBase& com) {
        while (running) {
            if (handleTimeout(lcd, com) || handleEmptyDisplay(lcd)) continue;

            // The reason for using this here is that the equals function
            // relies on strcmp, which cannot properly handle escape
            // sequences. As a result, it may fail to compare strings
            // correctly.
            if (compareIgnoringEscapeSequences(prevStr, lcd.color_str)) {
                delayWithTimeTracking(pdMS_TO_TICKS(10));
            } else {
                lcd.drawBlack();
                prevStr = lcd.color_str;
                lcd.printColorText(lcd.color_str);
            }
            delayWithTimeTracking(pdMS_TO_TICKS(10));
        }
    }

    static void startTaskImpl(void* _params) {
        auto* params = static_cast<std::tuple<Mode*, PrimitiveLCD*, CommunicationBase*>*>(_params);
        Mode* _this = std::get<0>(*params);
        PrimitiveLCD* lcd = std::get<1>(*params);
        CommunicationBase* com = std::get<2>(*params);
        _this->task(*lcd, *com);
        delete params;
        vTaskDelete(NULL);
    }

    virtual BaseType_t createTask(uint8_t xCoreID, PrimitiveLCD& lcd, CommunicationBase& com) {
        running = true;
        auto* params = new std::tuple<Mode*, PrimitiveLCD*, CommunicationBase*>(this, &lcd, &com);
        // Increasing the stack size (2048 -> 4096) prevents the following assertion:
        // assert failed: heap_caps_free heap_caps.c:381 (heap != NULL && "free() target pointer is
        // outside heap areas")
        this->xCoreID = xCoreID;
        return xTaskCreatePinnedToCore(this->startTaskImpl, getName().c_str(), 8192, params, 1,
                                       &taskHandle, xCoreID);
    }

    bool handleTimeout(PrimitiveLCD& lcd, CommunicationBase& com) {
        if (com.checkTimeout()) {
            lcd.drawNoDataReceived();
            lcd.printColorText(getName() + "\n");
            delayWithTimeTracking(pdMS_TO_TICKS(500));
            prevStr = "";
            return true;
        } else {
            // Short delay to avoiding watch-dog timer reset
            delayWithTimeTracking(pdMS_TO_TICKS(1 / portTICK_PERIOD_MS));
        }
        return false;
    }

    bool handleEmptyDisplay(PrimitiveLCD& lcd) {
        if (lcd.color_str.isEmpty()) {
            String waitStr = "Waiting for " + getName();
            lcd.drawBlack();
            lcd.printColorText(waitStr);
            delayWithTimeTracking(pdMS_TO_TICKS(500));
            return true;
        }
        return false;
    }

    ModeType::Name getModeType() const { return name; }

    bool isSkippable() const { return _isSkippable; }

protected:
    String prevStr;
    bool running;
    ModeType::Name name;
    bool _isSkippable;
};

#endif  // MODE_H
