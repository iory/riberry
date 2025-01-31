#ifndef MUTEX_HELPER_H
#define MUTEX_HELPER_H

#if defined(ESP32)
    #include <freertos/FreeRTOS.h>
    #include <freertos/semphr.h>
class MutexHelper {
public:
    MutexHelper() { mutex_ = xSemaphoreCreateMutex(); }

    ~MutexHelper() {
        if (mutex_) {
            vSemaphoreDelete(mutex_);
        }
    }

    void lock() {
        if (mutex_) {
            xSemaphoreTake(mutex_, portMAX_DELAY);
        }
    }

    void unlock() {
        if (mutex_) {
            xSemaphoreGive(mutex_);
        }
    }

private:
    SemaphoreHandle_t mutex_;
};

#else  // Non-ESP32 environment
class MutexHelper {
public:
    MutexHelper() {}

    ~MutexHelper() {}

    void lock() { noInterrupts(); }

    void unlock() { interrupts(); }
};
#endif

#endif  // MUTEX_HELPER_H
