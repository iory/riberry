#include <atom_s3_mode_manager.h>

AtomS3ModeManager* AtomS3ModeManager::instance = nullptr;

std::vector<Mode*> AtomS3ModeManager::allModes = {};
int AtomS3ModeManager::current_mode_index = 0;

AtomS3ModeManager::AtomS3ModeManager(AtomS3LCD &lcd, AtomS3Button &button, AtomS3I2C &i2c)
  : atoms3lcd(lcd), atoms3button(button), atoms3i2c(i2c)
{
  instance = this;
  createTask(0);
}

void AtomS3ModeManager::task(void *parameter) {
  if (instance == nullptr) {
    return;
  }

  instance->atoms3lcd.printWaitMessage(instance->atoms3i2c.i2c_slave_addr);
  while (true) {
    // Check if Mode is forced to change
    bool isModeForced = false;
    int forced_mode_index;
    for (int i = 0; i < allModes.size(); i++) {
      if (allModes[i]->getModeName() == instance->atoms3i2c.forcedMode) {
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
      int next_mode_index = (current_mode_index + 1) % allModes.size();
      instance->changeMode(current_mode_index, next_mode_index);
      current_mode_index = next_mode_index;
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void AtomS3ModeManager::createTask(uint8_t xCoreID) {
  xTaskCreatePinnedToCore(task, "Mode Manager Task", 2048, this, 24, NULL, xCoreID);
}

void AtomS3ModeManager::initializeAllModes(uint8_t xCoreID) {
  // Start user-defined mode
  for (int i = 0; i < allModes.size(); i++) {
    if (!allModes[i]->isTaskCreated()) {
      allModes[i]->createTask(xCoreID);
    }
    allModes[i]->suspendTask();
  }
}

void AtomS3ModeManager::startCurrentMode() {
  allModes[current_mode_index]->resumeTask();
}

void AtomS3ModeManager::changeMode(int suspend_mode_index, int resume_mode_index) {
    // Suspend
    allModes[suspend_mode_index]->suspendTask();
    // Transition
    allModes[suspend_mode_index]->waitForTaskSuspended();
    instance->atoms3i2c.stopReceiveEvent();
    instance->atoms3lcd.drawBlack();
    instance->atoms3lcd.printMessage("Wait for mode switch ...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    instance->atoms3lcd.resetLcdData();
    // Resume
    allModes[resume_mode_index]->resumeTask();
    instance->atoms3i2c.startReceiveEvent();
}

void AtomS3ModeManager::addMode(Mode &mode) {
  allModes.push_back(&mode);
}
