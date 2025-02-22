#ifndef CPU_USAGE_MONITOR_H
#define CPU_USAGE_MONITOR_H

#include <vector>

#include "esp_timer.h"
#include "execution_timer.h"

class CPUUsageMonitor {
public:
    CPUUsageMonitor(std::vector<ExecutionTimer*>& timers)
        : monitoredTimers(timers), lastTotalTime(esp_timer_get_time()) {
        previousExecutionTimes.resize(timers.size(), 0);
    }

    void calculateCPUUsage() {
        uint64_t currentTime = esp_timer_get_time();
        uint64_t totalElapsedTime = currentTime - lastTotalTime;
        lastTotalTime = currentTime;

        uint64_t totalExecutionTime = 0;
        std::vector<uint64_t> executionDiffs(monitoredTimers.size());

        for (size_t i = 0; i < monitoredTimers.size(); i++) {
            uint64_t currentExecutionTime = monitoredTimers[i]->getExecutionTime();
            executionDiffs[i] = currentExecutionTime - previousExecutionTimes[i];
            previousExecutionTimes[i] = currentExecutionTime;
            totalExecutionTime += executionDiffs[i];
        }

        uint64_t idleTime = (totalElapsedTime >= totalExecutionTime)
                                    ? (totalElapsedTime - totalExecutionTime)
                                    : 0;

        if (totalElapsedTime > 0) {
            for (size_t i = 0; i < monitoredTimers.size(); i++) {
                int cpuUsage = ((float)executionDiffs[i] / totalElapsedTime) * 100.0;
                USBSerial.printf("Task %s: CPU Usage = %d\n", monitoredTimers[i]->getName().c_str(),
                                 cpuUsage);
            }
            int idleUsage = ((float)idleTime / totalElapsedTime) * 100.0;
            USBSerial.printf("Idle CPU Usage = %d\n", idleUsage);
        }
    }

private:
    std::vector<ExecutionTimer*>& monitoredTimers;
    std::vector<uint64_t> previousExecutionTimes;
    uint64_t lastTotalTime;
};

#endif  // CPU_USAGE_MONITOR_H
