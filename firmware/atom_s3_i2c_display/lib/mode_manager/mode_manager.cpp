#include <mode_manager.h>

ModeManager *ModeManager::instance = nullptr;

std::vector<Mode *> ModeManager::selectedModes = {};
String ModeManager::selectedModesStr = "";
int ModeManager::current_mode_index = 0;
const std::vector<Mode *> *ModeManager::allModes = nullptr;

ModeManager::ModeManager(PrimitiveLCD &lcd,
                         ButtonManager &button,
                         CommunicationBase &i2c,
                         const std::vector<Mode *> &modes)
    : lcd(lcd), button_manager(button), comm(i2c), ExecutionTimer("Mode Manager") {
    instance = this;
    allModes = &modes;
}

void ModeManager::task(void *parameter) {
    if (instance == nullptr) {
        return;
    }

    while (true) {
        // Check if selected Modes are changed
        if (!selectedModesStr.equals(instance->comm.selectedModesStr)) {
            selectedModesStr = instance->comm.selectedModesStr;
            char **selectedModesStrList = (char **)malloc(allModes->size() * sizeof(char *));
            if (selectedModesStrList == nullptr) {
                Serial.println("Failed to allocate memory for selectedModesStrList.");
                return;
            }
            int modeCount = instance->comm.splitString(selectedModesStr, ',', selectedModesStrList,
                                                       allModes->size());
            // Check if all selectedModesStrList names are valid
            bool allModesFound = true;
            std::vector<int> invalidModeIndices;
            for (int i = 0; i < modeCount; i++) {
                bool modeFound = false;
                for (Mode *mode : *allModes) {
                    if (mode->getName().equals(String(selectedModesStrList[i]))) {
                        modeFound = true;
                        break;
                    }
                }
                if (!modeFound) {
                    allModesFound = false;
                    invalidModeIndices.push_back(i);
                }
            }
            if (!allModesFound) {
                instance->lcd.drawBlack();
                instance->lcd.printColorText("Invalid mode name found\n");
                for (int i : invalidModeIndices) {
                    instance->lcd.printColorText(String(selectedModesStrList[i]));
                    instance->lcd.printColorText("\n");
                }
                instance->delayWithTimeTracking(pdMS_TO_TICKS(500));
                continue;
            }
            // All selectedModesStrList name are valid, so suspend and add modes
            instance->suspendSelectedModes();
            for (int i = 0; i < modeCount; i++) {
                for (Mode *mode : *allModes) {
                    if (mode->getName().equals(String(selectedModesStrList[i]))) {
                        instance->addSelectedMode(*mode);
                    }
                }
            }
            free(selectedModesStrList);
            // Start the first task
            current_mode_index = 0;
            instance->startCurrentMode();
        }
        // If no mode is added, do nothing
        if (selectedModes.size() == 0) {
            instance->lcd.drawBlack();
            instance->lcd.printColorText("Waiting for modes to be added...\n");
            instance->delayWithTimeTracking(pdMS_TO_TICKS(500));
            continue;
        }
        // Check if Mode is forced to change
        bool isModeForced = false;
        int forced_mode_index;
        for (int i = 0; i < selectedModes.size(); i++) {
            if (selectedModes[i]->getName().equals(instance->comm.forcedMode)) {
                isModeForced = true;
                forced_mode_index = i;
                break;
            }
        }
        // Force mode change
        if (isModeForced) {
            instance->changeMode(current_mode_index, forced_mode_index);
            current_mode_index = forced_mode_index;
            instance->comm.forcedMode = "";
        }
        // Change mode by long click
        if (instance->button_manager.wasLongPressed()) {
            int next_mode_index = (current_mode_index + 1) % selectedModes.size();
            instance->changeMode(current_mode_index, next_mode_index);
            current_mode_index = next_mode_index;
        }
        instance->delayWithTimeTracking(pdMS_TO_TICKS(500));
    }
}

void ModeManager::createTask(uint8_t xCoreID) {
    this->xCoreID = xCoreID;
    xTaskCreatePinnedToCore(task, "Mode Manager Task", 2048, this, 24, NULL, xCoreID);
}

void ModeManager::startCurrentMode() {
    if (selectedModes.size() == 0) return;
    comm.setRequestStr(selectedModes[current_mode_index]->getName());
    uint8_t xCoreID = 1;
    selectedModes[current_mode_index]->createTask(xCoreID, lcd, comm);
}

void ModeManager::stopCurrentMode() {
    if (selectedModes.size() == 0) return;
    selectedModes[current_mode_index]->suspendTask();
}

bool ModeManager::isValidIndex(const std::vector<Mode *> &vec, int index) {
    return index >= 0 && index < static_cast<int>(vec.size());
}

void ModeManager::changeMode(int suspend_mode_index, int resume_mode_index) {
    if (!isValidIndex(selectedModes, suspend_mode_index) ||
        !isValidIndex(selectedModes, resume_mode_index))
        return;
    // Suspend
    // deleteTask may not release heap memory, so use suspendTask instead.
    instance->lcd.lockLcd();
    instance->lcd.drawBlack();
    instance->lcd.printColorText("Suspend task\n");
    // When switching modes, ModeManager may suspend the mode before the mode releases lcd
    // mutex. This deadlock causes lcd freeze. To avoid deadlock, mutex must be taken by
    // ModeManager before suspending the mode task.
    selectedModes[suspend_mode_index]->suspendTask();
    selectedModes[suspend_mode_index]->waitForTaskSuspended();
    instance->lcd.unlockLcd();

    // Transition
    instance->lcd.printColorText("Success fully suspend task\n");
    instance->comm.stopReceiveEvent();
    instance->lcd.drawBlack();
    instance->lcd.printColorText("Wait for mode switch ...\n");
    instance->lcd.setTextSize(DEFAULT_TEXT_SIZE);
    instance->lcd.resetLcdData();
    // Resume
    comm.setRequestStr(selectedModes[resume_mode_index]->getName());
    uint8_t xCoreID = 1;
    selectedModes[resume_mode_index]->resumeTask(xCoreID, lcd, comm);
    instance->comm.startReceiveEvent();
}

void ModeManager::addSelectedMode(Mode &mode) { selectedModes.push_back(&mode); }

void ModeManager::deleteSelectedModes() {
    // Before deleting modes, all modes must be suspended
    for (int i = 0; i < selectedModes.size(); i++) {
        selectedModes[i]->deleteTask();
    }
    selectedModes.clear();
}

void ModeManager::suspendSelectedModes() {
    // Before deleting modes, all modes must be suspended
    for (int i = 0; i < selectedModes.size(); i++) {
        selectedModes[i]->suspendTask();
    }
    selectedModes.clear();
}
