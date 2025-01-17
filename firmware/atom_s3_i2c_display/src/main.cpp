#include <vector>
#include <primitive_lcd.h>
#include <communication_base.h>
#include <button_managers.h>

#include <display_information_mode.h>
#include <display_qrcode_mode.h>
#include <display_image_mode.h>
#include <display_battery_graph_mode.h>
#include <display_odom_mode.h>
#include <servo_control_mode.h>
#include <pressure_control_mode.h>
#include <teaching_mode.h>

#include <atom_s3_mode_manager.h>

#ifdef ATOM_S3
  std::vector<int> pins = {41};
#elif defined(USE_M5STACK_BASIC)
  std::vector<int> pins = {39, 38, 37};
#else
  std::vector<int> pins = {41};
#endif

ButtonManagers button_managers(pins);
PrimitiveLCD lcd;
CommunicationBase comm(lcd, button_managers);

// Define all available modes
DisplayInformationMode display_information_mode(lcd, comm);
DisplayQRcodeMode display_qrcode_mode(lcd, comm);
DisplayImageMode display_image_mode(lcd, comm);
DisplayBatteryGraphMode display_battery_graph_mode(lcd, comm);
DisplayOdomMode display_odom_mode(lcd, comm);
ServoControlMode servo_control_mode(lcd, comm);
PressureControlMode pressure_control_mode(lcd, comm);
TeachingMode teaching_mode(lcd, comm);
const std::vector<Mode*> allModes =
  {&display_information_mode, &display_qrcode_mode, &display_image_mode, &display_battery_graph_mode,
   &display_odom_mode,
   &servo_control_mode, &pressure_control_mode, &teaching_mode,
  };

AtomS3ModeManager atoms3modemanager(lcd, button_managers, comm, allModes);

void setup() {
  button_managers.createTask(0);
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
