#include <atom_s3_lcd.h>
#include <atom_s3_i2c.h>
#include <atom_s3_button.h>

#include <display_information_mode.h>
#include <display_qrcode_mode.h>
#include <display_image_mode.h>
#include <display_battery_graph_mode.h>
#include <servo_control_mode.h>
#include <pressure_control_mode.h>
#include <teaching_mode.h>

#include <atom_s3_mode_manager.h>

AtomS3Button atoms3button;
AtomS3LCD atoms3lcd;
AtomS3I2C atoms3i2c(atoms3lcd, atoms3button);

// Define all available modes
DisplayInformationMode display_information_mode(atoms3lcd, atoms3i2c);
DisplayQRcodeMode display_qrcode_mode(atoms3lcd, atoms3i2c);
DisplayImageMode display_image_mode(atoms3lcd, atoms3i2c);
DisplayBatteryGraphMode display_battery_graph_mode(atoms3lcd, atoms3i2c);
ServoControlMode servo_control_mode(atoms3lcd, atoms3i2c);
PressureControlMode pressure_control_mode(atoms3lcd, atoms3i2c);
TeachingMode teaching_mode(atoms3lcd, atoms3i2c);
const std::vector<Mode*> allModes =
  {&display_information_mode, &display_qrcode_mode, &display_image_mode, &display_battery_graph_mode,
   &servo_control_mode, &pressure_control_mode, &teaching_mode,
  };

AtomS3ModeManager atoms3modemanager(atoms3lcd, atoms3button, atoms3i2c, allModes);

void setup() {
  atoms3button.createTask(0);
  atoms3i2c.createTask(0);
  atoms3modemanager.createTask(0);
  // By default, DisplayInformationMode and DisplayQRcodeMode are added
  atoms3modemanager.addSelectedMode(display_information_mode);
  atoms3modemanager.addSelectedMode(display_qrcode_mode);
  atoms3modemanager.initializeSelectedModes();
  atoms3modemanager.startCurrentMode();
}

void loop() {
}
