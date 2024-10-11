#include <atom_s3_lcd.h>
#include <atom_s3_i2c.h>
#include <atom_s3_button.h>
#include <display_information_mode.h>
#include <display_qrcode_mode.h>

// Common tasks for AtomS3
AtomS3Button atoms3button;
AtomS3LCD atoms3lcd;
AtomS3I2C atoms3i2c(atoms3lcd, atoms3button);

// User-defined modes:
DisplayInformationMode display_information_mode(atoms3lcd, atoms3i2c);
DisplayQRcodeMode display_qrcode_mode(atoms3lcd, atoms3i2c);
Mode* modes[] = { &display_information_mode, &display_qrcode_mode };
int current_mode_index = 0;
int num_modes = sizeof(modes) / sizeof(modes[0]);

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
  if (atoms3button.wasLongPressed()) {
    // Suspend
    modes[current_mode_index]->suspendTask();
    // Transition
    modes[current_mode_index]->waitForTaskSuspended();
    current_mode_index = (current_mode_index + 1) % num_modes;
    atoms3i2c.stopReceiveEvent();
    atoms3lcd.drawBlack();
    atoms3lcd.printMessage("Wait for mode switch ...");
    delay(1000);
    atoms3lcd.resetLcdData();
    // Resume
    modes[current_mode_index]->resumeTask();
    atoms3i2c.startReceiveEvent();
  }
  delay(500);
}
