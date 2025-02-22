#ifndef EXECUTION_TIMER_H
#define EXECUTION_TIMER_H

#include "Arduino.h"

class ExecutionTimer {
public:
    ExecutionTimer()
        : elapsedTime(0),
          startTime(esp_timer_get_time()),
          name("Unnamed"),
          taskHandle(NULL),
          xCoreID(-1) {}
    ExecutionTimer(const String &name)
        : elapsedTime(0),
          startTime(esp_timer_get_time()),
          name(name),
          taskHandle(NULL),
          xCoreID(-1) {}

    void delayWithTimeTracking(uint32_t ms) {
        uint64_t currentTime = esp_timer_get_time();
        elapsedTime += (currentTime - startTime);
        vTaskDelay(ms);
        startTime = esp_timer_get_time();
    }

    uint64_t getExecutionTime() const { return elapsedTime; }

    String getName() const { return name; }

    int getCoreID() const { return xCoreID; }

    void resetStats() {
        startTime = esp_timer_get_time();
        elapsedTime = 0LL;
    }

protected:
    uint64_t elapsedTime;
    uint64_t startTime;
    String name;
    int xCoreID;
    TaskHandle_t taskHandle;
};

#endif  // EXECUTION_TIMER_H
