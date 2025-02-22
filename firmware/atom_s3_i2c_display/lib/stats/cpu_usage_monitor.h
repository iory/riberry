#ifndef CPU_USAGE_MONITOR_H
#define CPU_USAGE_MONITOR_H

#include <vector>

#include "esp_timer.h"
#include "execution_timer.h"

class CPUUsageMonitor {
public:
    CPUUsageMonitor(std::vector<ExecutionTimer*>& timers)
        : monitoredTimers(timers), lastTime(esp_timer_get_time()) {}

    void calculateCPUUsage() {
        const uint64_t currentTime = esp_timer_get_time();
        const uint64_t totalElapsedTime = currentTime - lastTime;
        lastTime = currentTime;

        uint64_t totalExecutionTime = 0;
        std::vector<uint64_t> executionDiffs(monitoredTimers.size());

        for (size_t i = 0; i < monitoredTimers.size(); i++) {
            executionDiffs[i] = monitoredTimers[i]->getExecutionTime();
            totalExecutionTime += executionDiffs[i];
        }
        for (size_t i = 0; i < monitoredTimers.size(); i++) {
            monitoredTimers[i]->resetStats();
        }
        uint64_t idleTime = (totalElapsedTime >= totalExecutionTime)
                                    ? (totalElapsedTime - totalExecutionTime)
                                    : 0;

        if (totalElapsedTime > 0) {
            for (size_t i = 0; i < monitoredTimers.size(); i++) {
                USBSerial.printf("Task %s: Execution Time = %llu, totalElapsedTime = %llu \n",
                                 monitoredTimers[i]->getName().c_str(),
                                 (unsigned long long)executionDiffs[i],
                                 (unsigned long long)totalElapsedTime);
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
    uint64_t lastTime;
};

#endif  // CPU_USAGE_MONITOR_H
