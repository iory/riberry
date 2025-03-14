#ifndef SYSTEM_DEBUG_MODE_H
#define SYSTEM_DEBUG_MODE_H

#include <color.h>
#include <mode.h>

class SystemDebugMode : public Mode {
public:
    SystemDebugMode(bool isSkippable = false) : Mode(ModeType::SYSTEM_DEBUG, isSkippable) {
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
                delayWithTimeTracking(pdMS_TO_TICKS(10));
            } else {
                lcd.drawBlack();
                lcd.printColorText(str);
                prevStr = str;
            }
            delayWithTimeTracking(pdMS_TO_TICKS(1000));
        }
    }

    virtual void suspendTask() {
        previousFreeHeap = currentFreeHeap;
        Mode::suspendTask();
    }

    String checkMemoryLeak() {
        String message = "";
        char buf[120];

        sprintf(buf, "Free heap [%sKiB%s]\n", Color::Foreground::YELLOW, Color::Foreground::RESET);
        message += String(buf);
        if (previousFreeHeap == 0) {  // First time
            message += " Prev  Unknown\n";
        } else {
            sprintf(buf, " Prev  %s%d%s\n", Color::Foreground::GREEN, previousFreeHeap >> 10,
                    Color::Foreground::RESET);
            message += String(buf);
        }

        currentFreeHeap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
        sprintf(buf, " Now   %s%d%s\n Total %s%d%s\n", Color::Foreground::GREEN,
                currentFreeHeap >> 10, Color::Foreground::RESET, Color::Foreground::GREEN,
                totalFreeHeap >> 10, Color::Foreground::RESET);
        message += String(buf);

        if (currentFreeHeap < previousFreeHeap) {
            sprintf(buf, "\nMemory may leak. Lost %s%d%s %sbytes%s compared to previous run.\n",
                    Color::Foreground::GREEN, previousFreeHeap - currentFreeHeap,
                    Color::Foreground::RESET, Color::Foreground::YELLOW, Color::Foreground::RESET);
            message += String(buf);
        }

        return message;
    }

    size_t previousFreeHeap;
    size_t currentFreeHeap;
    size_t totalFreeHeap;
};

#endif  // SYSTEM_DEBUG_MODE_H
