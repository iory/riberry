#include <atom_s3_lcd.h>
#include <atom_s3_i2c.h>
#include <atom_s3_button.h>

#include <atom_s3_mode_manager.h>
#include <display_information_mode.h>
#include <display_qrcode_mode.h>
#include <display_image_mode.h>

AtomS3Button atoms3button;
AtomS3LCD atoms3lcd;
AtomS3I2C atoms3i2c(atoms3lcd, atoms3button);
AtomS3ModeManager atoms3modemanager(atoms3lcd, atoms3button, atoms3i2c);

DisplayInformationMode display_information_mode(atoms3lcd, atoms3i2c);
DisplayQRcodeMode display_qrcode_mode(atoms3lcd, atoms3i2c);
DisplayImageMode display_image_mode(atoms3lcd, atoms3i2c);

void setup() {
  atoms3modemanager.addMode(display_information_mode);
  atoms3modemanager.addMode(display_qrcode_mode);
  atoms3modemanager.addMode(display_image_mode);

  // Start one of the user-defined modes
  uint8_t eachModeCoreID = 1;
  atoms3modemanager.initializeAllModes(eachModeCoreID);
  atoms3modemanager.startCurrentMode();
}

void loop() {
}
