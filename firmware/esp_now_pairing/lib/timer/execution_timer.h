#ifndef EXECUTION_TIMER_H
#define EXECUTION_TIMER_H

#include "Arduino.h"

class ExecutionTimer {
public:
    ExecutionTimer() : elapsedTime(0), startTime(0) {}

    void delayWithTimeTracking(uint32_t ms) {
        uint64_t currentTime = micros();
        elapsedTime += (currentTime - startTime);
        startTime = currentTime;
        vTaskDelay(pdMS_TO_TICKS(ms));
    }

    uint64_t getExecutionTime() const { return elapsedTime; }

protected:
    uint64_t elapsedTime;
    uint64_t startTime;
};

#endif  // EXECUTION_TIMER_H
