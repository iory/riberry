#ifndef CPU_USAGE_MONITOR_H
#define CPU_USAGE_MONITOR_H

#include <string>
#include <unordered_map>
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

        std::unordered_map<int, uint64_t> coreTotalExecTime;
        std::unordered_map<int, std::vector<std::pair<ExecutionTimer*, uint64_t>>> coreTimers;

        for (size_t i = 0; i < monitoredTimers.size(); ++i) {
            int coreID = monitoredTimers[i]->getCoreID();
            uint64_t execTime = monitoredTimers[i]->getExecutionTime();
            coreTotalExecTime[coreID] += execTime;
            coreTimers[coreID].push_back({monitoredTimers[i], execTime});
        }

        for (size_t i = 0; i < monitoredTimers.size(); ++i) {
            monitoredTimers[i]->resetStats();
        }

        USBSerial.printf("===== CPU Usage Monitor =====\n");
        USBSerial.printf("Elapsed Time: %llu us\n\n", totalElapsedTime);

        for (int coreID = 0; coreID <= 1; coreID++) {
            auto it = coreTimers.find(coreID);
            if (it == coreTimers.end()) {
                USBSerial.printf("Core %d: No tasks.\n\n", coreID);
                continue;
            }
            const auto& timers = it->second;
            uint64_t coreExecTime = coreTotalExecTime[coreID];
            uint64_t idleTime =
                    (totalElapsedTime >= coreExecTime) ? (totalElapsedTime - coreExecTime) : 0;
            USBSerial.printf("Core %d:\n", coreID);
            for (const auto& timerPair : timers) {
                ExecutionTimer* timer = timerPair.first;
                uint64_t execTime = timerPair.second;
                float cpuUsage = (totalElapsedTime > 0)
                                         ? ((float)execTime / totalElapsedTime) * 100.0f
                                         : 0.0f;
                USBSerial.printf("  %-25s : %.2f%%\n", timer->getName().c_str(), cpuUsage);
            }
            float idleUsage =
                    (totalElapsedTime > 0) ? ((float)idleTime / totalElapsedTime) * 100.0f : 0.0f;
            USBSerial.printf("  %-25s : %.2f%%\n", "Idle CPU Usage", idleUsage);
            USBSerial.printf("\n");
        }
    }

private:
    std::vector<ExecutionTimer*>& monitoredTimers;
    uint64_t lastTime;
};

#endif  // CPU_USAGE_MONITOR_H
