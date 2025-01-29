
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
#include <pairing_mode.h>

#include <mode_manager.h>
#include <pairing.h>

#include <Arduino.h>
#include <HardwareSerial.h>
ButtonManager button_manager;
PrimitiveLCD lcd;
Pairing pairing;

#if defined(PAIRING_TYPE) && PAIRING_TYPE == 0
  const char *main_or_secondary = "Main";
#else
  const char *main_or_secondary = "Secondary";
#endif

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
    CommunicationBase comm(lcd, button_manager, pairing, &USBSerial, main_or_secondary);
  #else
    #include <WireSlave.h>
    CommunicationBase comm(lcd, button_manager, pairing, &WireSlave, main_or_secondary);
  #endif

#elif USE_M5STACK_BASIC
  CommunicationBase comm(lcd, button_manager, pairing, &Serial, main_or_secondary);
#endif

// Define all available modes
DisplayInformationMode display_information_mode;
DisplayQRcodeMode display_qrcode_mode;
DisplayImageMode display_image_mode;
DisplayBatteryGraphMode display_battery_graph_mode;
DisplayOdomMode display_odom_mode;
ServoControlMode servo_control_mode;
PressureControlMode pressure_control_mode;
TeachingMode teaching_mode;
PairingMode pairing_mode(button_manager, pairing);
const std::vector<Mode*> allModes =
  {&display_information_mode, &display_qrcode_mode, &display_image_mode, &display_battery_graph_mode,
   &display_odom_mode,
   &servo_control_mode, &pressure_control_mode, &teaching_mode,
   &pairing_mode,
  };

ModeManager modemanager(lcd, button_manager, comm, allModes);

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
  modemanager.createTask(0);
  // By default, DisplayInformationMode and DisplayQRcodeMode are added
  modemanager.addSelectedMode(display_information_mode);
  modemanager.addSelectedMode(display_qrcode_mode);
  modemanager.addSelectedMode(pairing_mode);
  modemanager.initializeSelectedModes();
  modemanager.startCurrentMode();
}

void loop() {
}
