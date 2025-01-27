
#include <primitive_lcd.h>
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
#include <pairing_mode.h>

#include <Arduino.h>
#include <HardwareSerial.h>
ButtonManager button_manager;
PrimitiveLCD lcd;
#ifdef ATOM_S3
  #ifdef I2C_ADDR
    static constexpr int i2c_slave_addr = I2C_ADDR; /**< I2C slave address for communication. */
  #else
    static constexpr int i2c_slave_addr = 0x42; /**< I2C slave address for communication. */
  #endif // end of I2C_ADDR

  #ifdef USE_GROVE
    static constexpr int sda_pin = 2; /**< I2C SDA pin for GROVE mode. */
    static constexpr int scl_pin = 1; /**< I2C SCL pin for GROVE mode. */
  #else
    static constexpr int sda_pin = 38; /**< I2C SDA pin for default mode. */
    static constexpr int scl_pin = 39; /**< I2C SCL pin for default mode. */
  #endif // end of USE_GROVE

  #ifdef USE_USB_SERIAL
    CommunicationBase comm(lcd, button_manager, &USBSerial);
  #else
    #include <WireSlave.h>
    CommunicationBase comm(lcd, button_manager, &WireSlave);
  #endif

#elif USE_M5STACK_BASIC
  CommunicationBase comm(lcd, button_manager, &Serial);
#endif

// Define all available modes
DisplayInformationMode display_information_mode(lcd, comm);
DisplayQRcodeMode display_qrcode_mode(lcd, comm);
DisplayImageMode display_image_mode(lcd, comm);
DisplayBatteryGraphMode display_battery_graph_mode(lcd, comm);
DisplayOdomMode display_odom_mode(lcd, comm);
ServoControlMode servo_control_mode(lcd, comm);
PressureControlMode pressure_control_mode(lcd, comm);
TeachingMode teaching_mode(lcd, comm);
PairingMode pairing_mode(lcd, comm);
const std::vector<Mode*> allModes =
  {&display_information_mode, &display_qrcode_mode, &display_image_mode, &display_battery_graph_mode,
   &display_odom_mode,
   &servo_control_mode, &pressure_control_mode, &teaching_mode,
   &pairing_mode,
  };

AtomS3ModeManager atoms3modemanager(lcd, button_manager, comm, allModes);

void setup() {
#ifdef ATOM_S3
  #ifdef USE_USB_SERIAL
    USBSerial.begin(115200);
    bool success = true;
  #else
    bool success = WireSlave.begin(sda_pin, scl_pin, i2c_slave_addr, 200, 100);
  #endif
#elif USE_M5STACK_BASIC
  bool success = true;
  Serial.begin(115200, SERIAL_8N1, 16, 17);
#endif
  if (!success) {
    lcd.printColorText("I2C slave init failed\n");
    while (1) vTaskDelay(pdMS_TO_TICKS(100));;
  }
  button_manager.createTask(0);
  comm.createTask(0);
  atoms3modemanager.createTask(0);
  // By default, DisplayInformationMode and DisplayQRcodeMode are added
  atoms3modemanager.addSelectedMode(display_information_mode);
  atoms3modemanager.addSelectedMode(display_qrcode_mode);
  atoms3modemanager.addSelectedMode(pairing_mode);
  atoms3modemanager.initializeSelectedModes();
  atoms3modemanager.startCurrentMode();
}

void loop() {
}
