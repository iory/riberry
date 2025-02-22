#ifndef CPU_USAGE_MONITOR_H
#define CPU_USAGE_MONITOR_H

#include <string>
#include <unordered_map>
#include <vector>

#include "color.h"
#include "esp_timer.h"
#include "execution_timer.h"

constexpr int CPU_MID_RANGE = 20;
constexpr int CPU_HIGH_RANGE = 32;
constexpr int BAR_LENGTH = 40;

class CPUUsageMonitor {
public:
    CPUUsageMonitor(std::vector<ExecutionTimer*>& timers)
        : monitoredTimers(timers), lastTime(esp_timer_get_time()) {}

    void printCpuUsageBar(float cpuUsage) {
        int bars = (int)((cpuUsage / 100.0f) * BAR_LENGTH);
        USBSerial.printf("  [");

        for (int s = 0; s < BAR_LENGTH; s++) {
            if (s < bars) {
                if (s < CPU_MID_RANGE) {
                    USBSerial.printf("%s|", Color::Foreground::GREEN);
                } else if (s < CPU_HIGH_RANGE) {
                    USBSerial.printf("%s|", Color::Foreground::YELLOW);
                } else {
                    USBSerial.printf("%s|", Color::Foreground::RED);
                }
            } else {
                USBSerial.printf(" ");
            }
        }
        USBSerial.printf("%s] %03.2f%%\r\n", Color::Foreground::RESET, cpuUsage);
    }

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
                USBSerial.printf("  %-25s : ", timer->getName().c_str());
                printCpuUsageBar(cpuUsage);
            }
            float idleUsage =
                    (totalElapsedTime > 0) ? ((float)idleTime / totalElapsedTime) * 100.0f : 0.0f;
            USBSerial.printf("  %-25s : ", "Idle CPU Usage");
            printCpuUsageBar(idleUsage);
            USBSerial.printf("\n");
        }
    }

private:
    std::vector<ExecutionTimer*>& monitoredTimers;
    uint64_t lastTime;
};

#endif  // CPU_USAGE_MONITOR_H
