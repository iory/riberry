#include <atom_s3_lcd.h>
#include <communication_base.h>
#include <button_manager.h>

#include <display_information_mode.h>
#include <display_qrcode_mode.h>
#include <display_image_mode.h>
#include <display_battery_graph_mode.h>
#include <display_odom_mode.h>
#include <servo_control_mode.h>
#include <pressure_control_mode.h>
#include <teaching_mode.h>

#include <atom_s3_mode_manager.h>

ButtonManager button_manager;
AtomS3LCD atoms3lcd;
CommunicationBase comm(atoms3lcd, button_manager);

// Define all available modes
DisplayInformationMode display_information_mode(atoms3lcd, comm);
DisplayQRcodeMode display_qrcode_mode(atoms3lcd, comm);
DisplayImageMode display_image_mode(atoms3lcd, comm);
DisplayBatteryGraphMode display_battery_graph_mode(atoms3lcd, comm);
DisplayOdomMode display_odom_mode(atoms3lcd, comm);
ServoControlMode servo_control_mode(atoms3lcd, comm);
PressureControlMode pressure_control_mode(atoms3lcd, comm);
TeachingMode teaching_mode(atoms3lcd, comm);
const std::vector<Mode*> allModes =
  {&display_information_mode, &display_qrcode_mode, &display_image_mode, &display_battery_graph_mode,
   &display_odom_mode,
   &servo_control_mode, &pressure_control_mode, &teaching_mode,
  };

AtomS3ModeManager atoms3modemanager(atoms3lcd, button_manager, comm, allModes);

void setup() {
  button_manager.createTask(0);
  comm.createTask(0);
  atoms3modemanager.createTask(0);
  // By default, DisplayInformationMode and DisplayQRcodeMode are added
  atoms3modemanager.addSelectedMode(display_information_mode);
  atoms3modemanager.addSelectedMode(display_qrcode_mode);
  atoms3modemanager.initializeSelectedModes();
  atoms3modemanager.startCurrentMode();
}

void loop() {
}
