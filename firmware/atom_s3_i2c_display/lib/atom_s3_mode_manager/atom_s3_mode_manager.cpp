#include <atom_s3_mode_manager.h>

AtomS3ModeManager* AtomS3ModeManager::instance = nullptr;

std::vector<Mode*> AtomS3ModeManager::selectedModes = {};
String AtomS3ModeManager::selectedModesStr = "";
int AtomS3ModeManager::current_mode_index = 0;
const std::vector<Mode*>* AtomS3ModeManager::allModes = nullptr;

AtomS3ModeManager::AtomS3ModeManager(AtomS3LCD &lcd, AtomS3Button &button, AtomS3I2C &i2c, const std::vector<Mode *> &modes)
  : atoms3lcd(lcd), atoms3button(button), atoms3i2c(i2c)
{
  instance = this;
  allModes = &modes;
}

void AtomS3ModeManager::task(void *parameter) {
  if (instance == nullptr) {
    return;
  }

  while (true) {
    // Check if selected Modes are changed
    if (!selectedModesStr.equals(instance->atoms3i2c.selectedModesStr)) {
      // Delete all modes
      instance->deleteSelectedModes();
      // Add selected modes
      selectedModesStr = instance->atoms3i2c.selectedModesStr;
      char** selectedModesStrList = (char**)malloc(allModes->size() * sizeof(char*));
      if (selectedModesStrList == nullptr) {
        // メモリ確保失敗時の処理
        Serial.println("Failed to allocate memory for selectedModesStrList.");
        return;
      }
      int modeCount = instance->atoms3i2c.splitString(selectedModesStr, ',', selectedModesStrList, allModes->size());
      for (int i = 0; i < modeCount; i++) {
        for (Mode *mode : *allModes) {
          if (mode->getModeName().equals(String(selectedModesStrList[i]))) {
            instance->addSelectedMode(*mode); // ポインタから参照に変換
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
      instance->atoms3lcd.drawBlack();
      instance->atoms3lcd.printColorText("Waiting for modes to be added...\n");
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    // Check if Mode is forced to change
    bool isModeForced = false;
    int forced_mode_index;
    for (int i = 0; i < selectedModes.size(); i++) {
      if (selectedModes[i]->getModeName().equals(instance->atoms3i2c.forcedMode)) {
        isModeForced = true;
        forced_mode_index = i;
        break;
      }
    }
    // Force mode change
    if (isModeForced) {
      instance->changeMode(current_mode_index, forced_mode_index);
      current_mode_index = forced_mode_index;
      instance->atoms3i2c.forcedMode = "";
    }
    // Change mode by long click
    if (instance->atoms3button.wasLongPressed()) {
      int next_mode_index = (current_mode_index + 1) % selectedModes.size();
      instance->changeMode(current_mode_index, next_mode_index);
      current_mode_index = next_mode_index;
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void AtomS3ModeManager::createTask(uint8_t xCoreID) {
  xTaskCreatePinnedToCore(task, "Mode Manager Task", 2048, this, 24, NULL, xCoreID);
}

void AtomS3ModeManager::initializeSelectedModes() {
  // Start user-defined mode
  uint8_t xCoreID = 1;
  for (int i = 0; i < selectedModes.size(); i++) {
    if (!selectedModes[i]->isTaskCreated()) {
      selectedModes[i]->createTask(xCoreID);
    }
    selectedModes[i]->suspendTask();
  }
}

void AtomS3ModeManager::startCurrentMode() {
  if (selectedModes.size() == 0)
    return;
  selectedModes[current_mode_index]->resumeTask();
}

void AtomS3ModeManager::stopCurrentMode() {
  if (selectedModes.size() == 0)
    return;
  selectedModes[current_mode_index]->suspendTask();
}

bool AtomS3ModeManager::isValidIndex(const std::vector<Mode*>& vec, int index) {
  // index が 0 以上 vec.size() 未満であれば true を返す
  return index >= 0 && index < static_cast<int>(vec.size());
}

void AtomS3ModeManager::changeMode(int suspend_mode_index, int resume_mode_index) {
  if (!isValidIndex(selectedModes, suspend_mode_index) || !isValidIndex(selectedModes, resume_mode_index))
      return;
  // Suspend
  selectedModes[suspend_mode_index]->suspendTask();
  // Transition
  selectedModes[suspend_mode_index]->waitForTaskSuspended();
  instance->atoms3i2c.stopReceiveEvent();
  instance->atoms3lcd.drawBlack();
  instance->atoms3lcd.printColorText("Wait for mode switch ...\n");
  vTaskDelay(pdMS_TO_TICKS(1000));
  instance->atoms3lcd.resetLcdData();
  // Resume
  selectedModes[resume_mode_index]->resumeTask();
  instance->atoms3i2c.startReceiveEvent();
}

void AtomS3ModeManager::addSelectedMode(Mode &mode) {
  selectedModes.push_back(&mode);
}

void AtomS3ModeManager::deleteSelectedModes() {
  // Before deleting modes, all modes must be suspended
  for (int i = 0; i < selectedModes.size(); i++) {
    if (!selectedModes[i]->isTaskSuspended()) {
      selectedModes[i]->suspendTask();
    }
  }
  selectedModes.clear();
}
