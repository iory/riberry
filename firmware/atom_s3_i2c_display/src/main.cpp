#include <atom_s3_lcd.h>
#include <atom_s3_i2c.h>
#include <atom_s3_button.h>
#include <display_information_mode.h>
#include <display_qrcode_mode.h>
#include <display_image_mode.h>

// Common tasks for AtomS3
AtomS3Button atoms3button;
AtomS3LCD atoms3lcd;
AtomS3I2C atoms3i2c(atoms3lcd, atoms3button);

// User-defined modes:
DisplayInformationMode display_information_mode(atoms3lcd, atoms3i2c);
DisplayQRcodeMode display_qrcode_mode(atoms3lcd, atoms3i2c);
DisplayImageMode display_image_mode(atoms3lcd, atoms3i2c);
Mode* modes[] = { &display_information_mode, &display_qrcode_mode, &display_image_mode };
int current_mode_index = 0;
int num_modes = sizeof(modes) / sizeof(modes[0]);

void changeMode(int suspend_mode_index, int resume_mode_index) {
    // Suspend
    modes[suspend_mode_index]->suspendTask();
    // Transition
    modes[suspend_mode_index]->waitForTaskSuspended();
    atoms3i2c.stopReceiveEvent();
    atoms3lcd.drawBlack();
    atoms3lcd.printMessage("Wait for mode switch ...");
    delay(1000);
    atoms3lcd.resetLcdData();
    // Resume
    modes[resume_mode_index]->resumeTask();
    atoms3i2c.startReceiveEvent();
}

void setup() {
  atoms3lcd.printWaitMessage(atoms3i2c.i2c_slave_addr);

  // Start user-defined mode
  for (int i = 0; i < num_modes; i++) {
    uint8_t xCoreID = 1;
    modes[i]->createTask(xCoreID);
    modes[i]->suspendTask();
    }
  modes[current_mode_index]->resumeTask();
}

void loop() {
  // Check if Mode is forced to change
  bool isModeForced = false;
  int forced_mode_index;
  for (int i = 0; i < num_modes; i++) {
    if (modes[i]->getModeName() == atoms3i2c.forcedMode) {
      isModeForced = true;
      forced_mode_index = i;
      break;
    }
  }
  // Force mode change
  if (isModeForced) {
    changeMode(current_mode_index, forced_mode_index);
    current_mode_index = forced_mode_index;
    atoms3i2c.forcedMode = "";
  }
  // Change mode by long click
  if (atoms3button.wasLongPressed()) {
    int next_mode_index = (current_mode_index + 1) % num_modes;
    changeMode(current_mode_index, next_mode_index);
    current_mode_index = next_mode_index;
  }
  delay(500);
}
