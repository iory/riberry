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
    : lcd(lcd), button_manager(button), comm(i2c) {
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
            // Delete all modes
            instance->deleteSelectedModes();
            // Add selected modes
            selectedModesStr = instance->comm.selectedModesStr;
            char **selectedModesStrList = (char **)malloc(allModes->size() * sizeof(char *));
            if (selectedModesStrList == nullptr) {
                Serial.println("Failed to allocate memory for selectedModesStrList.");
                return;
            }
            int modeCount = instance->comm.splitString(selectedModesStr, ',', selectedModesStrList,
                                                       allModes->size());
            for (int i = 0; i < modeCount; i++) {
                for (Mode *mode : *allModes) {
                    if (mode->getModeName().equals(String(selectedModesStrList[i]))) {
                        instance->addSelectedMode(*mode);
                    }
                }
            }
            free(selectedModesStrList);
            // Start task
            current_mode_index = 0;
            instance->initializeSelectedModes();
            instance->startCurrentMode();
        }
        // If no mode is added, do nothing
        if (selectedModes.size() == 0) {
            instance->lcd.drawBlack();
            instance->lcd.printColorText("Waiting for modes to be added...\n");
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        // Check if Mode is forced to change
        bool isModeForced = false;
        int forced_mode_index;
        for (int i = 0; i < selectedModes.size(); i++) {
            if (selectedModes[i]->getModeName().equals(instance->comm.forcedMode)) {
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
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void ModeManager::createTask(uint8_t xCoreID) {
    xTaskCreatePinnedToCore(task, "Mode Manager Task", 2048, this, 24, NULL, xCoreID);
}

void ModeManager::initializeSelectedModes() {
    // Start user-defined mode
    uint8_t xCoreID = 1;
    for (int i = 0; i < selectedModes.size(); i++) {
        if (!selectedModes[i]->isTaskCreated()) {
            selectedModes[i]->createTask(xCoreID, lcd, comm);
        }
        selectedModes[i]->suspendTask();
    }
}

void ModeManager::startCurrentMode() {
    if (selectedModes.size() == 0) return;
    comm.setRequestStr(selectedModes[current_mode_index]->getModeName());
    selectedModes[current_mode_index]->resumeTask();
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
    instance->lcd.printColorText("Suspend task\n");
    selectedModes[suspend_mode_index]->suspendTask();
    // Transition
    instance->lcd.printColorText("WaitForTaskSuspended suspend\n");
    selectedModes[suspend_mode_index]->waitForTaskSuspended();
    instance->lcd.printColorText("Success fully delete task\n");
    instance->comm.stopReceiveEvent();
    instance->lcd.drawBlack();
    instance->lcd.printColorText("Wait for mode switch ...\n");
    instance->lcd.setTextSize(DEFAULT_TEXT_SIZE);
    vTaskDelay(pdMS_TO_TICKS(1000));
    instance->lcd.resetLcdData();
    // Resume
    comm.setRequestStr(selectedModes[resume_mode_index]->getModeName());
    selectedModes[resume_mode_index]->resumeTask();
    instance->comm.startReceiveEvent();
}

void ModeManager::addSelectedMode(Mode &mode) { selectedModes.push_back(&mode); }

void ModeManager::deleteSelectedModes() {
    // Before deleting modes, all modes must be suspended
    for (int i = 0; i < selectedModes.size(); i++) {
        selectedModes[i]->suspendTask();
        selectedModes[i]->waitForTaskSuspended();
    }
    selectedModes.clear();
}
