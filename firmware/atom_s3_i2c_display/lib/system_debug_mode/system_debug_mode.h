#ifndef SYSTEM_DEBUG_MODE_H
#define SYSTEM_DEBUG_MODE_H

#include <mode.h>

class SystemDebugMode : public Mode {
public:
    SystemDebugMode() : Mode("SystemDebugMode") {
        previousFreeHeap = 0;  // Unknown at first time
        totalFreeHeap = heap_caps_get_total_size(MALLOC_CAP_8BIT);
    }

private:
    void task(PrimitiveLCD &lcd, CommunicationBase &com) override {
        while (running) {
#ifdef ATOM_S3
            lcd.setTextSize(1.2);
#endif
            String str = "";
            str += checkMemoryLeak();
            if (prevStr.equals(str)) {
                vTaskDelay(pdMS_TO_TICKS(10));
            } else {
                lcd.drawBlack();
                lcd.printColorText(str);
                prevStr = str;
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    virtual void suspendTask() {
        previousFreeHeap = currentFreeHeap;
        Mode::suspendTask();
    }

    String checkMemoryLeak() {
        String message = "";
        char buf[120];

        message += "Free heap [bytes]\n";
        if (previousFreeHeap == 0) {  // First time
            message += " Prev  Unknown\n";
        } else {
            sprintf(buf, " Prev  %d\n", previousFreeHeap);
            message += String(buf);
        }

        currentFreeHeap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
        sprintf(buf, " Now   %d\n Total %d\n", currentFreeHeap, totalFreeHeap);
        message += String(buf);

        if (currentFreeHeap < previousFreeHeap) {
            sprintf(buf, "\nMemory may leak. Lost %d bytes compared to previous run.\n",
                    previousFreeHeap - currentFreeHeap);
            message += String(buf);
        }

        return message;
    }

    size_t previousFreeHeap;
    size_t currentFreeHeap;
    size_t totalFreeHeap;
};

#endif  // SYSTEM_DEBUG_MODE_H
