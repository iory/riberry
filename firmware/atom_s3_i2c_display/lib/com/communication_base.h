#ifndef COMMUNICATION_BASE_H
#define COMMUNICATION_BASE_H

#ifdef ATOM_S3
  #include <Wire.h>
  #include <WireSlave.h> // for i2c
#endif

#include <primitive_lcd.h>
#include <button_managers.h>

#include "packet.h"

class CommunicationBase {
public:
  virtual ~CommunicationBase() = default;

  /**
   * @brief Constructor for CommunicationBase, initializes the I2C class with references to the LCD and Button objects.
   *
   * @param lcd Reference to the PrimitiveLCD object.
   * @param button Reference to the ButtonManager object.
   */
  CommunicationBase(PrimitiveLCD &lcd, ButtonManagers &buttons);

  /**
   * @brief Creates and starts the I2C communication task on the specified core.
   *
   * @param xCoreID Core ID to run the task on (0 or 1).
   */
  void createTask(uint8_t xCoreID);

  /**
   * @brief Checks whether the I2C communication has timed out.

   *
   * @return true if the communication has timed out, false otherwise.
   */
  bool checkTimeout();

  int splitString(const String &input, char delimiter, char* output[], int maxParts);

  void stopReceiveEvent();
  void startReceiveEvent();
  static void setRequestStr(const String &str);

  static String forcedMode;
  static String selectedModesStr;

private:

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
#endif // end of ATOM_S3

  bool receiveEventEnabled;
  static CommunicationBase* instance; /**< Singleton instance of CommunicationBase for managing callbacks. */
  PrimitiveLCD &lcd; /**< Reference to the PrimitiveLCD object for displaying information. */
  ButtonManagers &button_managers; /**< Reference to the ButtonManager object for button interactions. */
  unsigned long lastReceiveTime = 0; /**< Last time data was received over I2C. */
  const unsigned long receiveTimeout = 15000; /**< Timeout duration for I2C communication (15 seconds). */

  static constexpr uint8_t jpegPacketHeader[3] = { 0xFF, 0xD8, 0xEA }; /**< JPEG image packet header identifier. */
  static constexpr uint8_t qrCodeHeader = 0x02; /**< QR code packet header identifier. */
  static constexpr uint8_t forceModeHeader[3] = { 0xFF, 0xFE, 0xFD }; /**< Force mode packet header identifier. */
  static constexpr uint8_t selectedModesHeader[3] = { 0xFC, 0xFB, 0xFA }; /**< Selected modes packet header identifier. */

  static String requestStr; /**< String to be sent on I2C request. */

  /**
   * @brief Updates the last receive time to the current time.
   */
  void updateLastReceiveTime();

  /**
   * @brief Called when data is received over communication.
   *
   * @param howMany Number of bytes received.
   */
  static void receiveEvent(int howMany);
  static void handleJpegPacket(const String& str);
  static void handleQrCodePacket(const String& str);
  static void handleForceModePacket(const String& str);
  static void handleSelectedModePacket(const String& str);

  /**
   * @brief Called when the master requests data from the I2C or UART slave.
   */
  static void requestEvent();

  /**
   * @brief The task that handles communication in the background.
   *
   * @param parameter Task parameter (unused).
   */
  static void task(void *parameter);
};

#endif
