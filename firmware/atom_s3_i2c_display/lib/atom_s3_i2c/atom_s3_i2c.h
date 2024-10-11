#ifndef ATOM_S3_I2C_H
#define ATOM_S3_I2C_H

#include <Wire.h>
#include <WireSlave.h>
#include <atom_s3_lcd.h>
#include <atom_s3_button.h>

/**
 * @brief Handles I2C communication for AtomS3, including receiving and sending data via I2C bus.
 */
class AtomS3I2C {
public:
  /**
   * @brief Constructor for AtomS3I2C, initializes the I2C class with references to the LCD and Button objects.
   *
   * @param lcd Reference to the AtomS3LCD object.
   * @param button Reference to the AtomS3Button object.
   */
  AtomS3I2C(AtomS3LCD &lcd, AtomS3Button &button);

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

#ifdef I2C_ADDR
  static constexpr int i2c_slave_addr = I2C_ADDR; /**< I2C slave address for communication. */
#else
  static constexpr int i2c_slave_addr = 0x42; /**< I2C slave address for communication. */
#endif

#ifdef USE_GROVE
  static constexpr int sda_pin = 2; /**< I2C SDA pin for GROVE mode. */
  static constexpr int scl_pin = 1; /**< I2C SCL pin for GROVE mode. */
#else
  static constexpr int sda_pin = 38; /**< I2C SDA pin for default mode. */
  static constexpr int scl_pin = 39; /**< I2C SCL pin for default mode. */
#endif

private:
  static AtomS3I2C* instance; /**< Singleton instance of AtomS3I2C for managing callbacks. */
  AtomS3LCD &atoms3lcd; /**< Reference to the AtomS3LCD object for displaying information. */
  AtomS3Button &atoms3button; /**< Reference to the AtomS3Button object for button interactions. */
  unsigned long lastReceiveTime = 0; /**< Last time data was received over I2C. */
  const unsigned long receiveTimeout = 15000; /**< Timeout duration for I2C communication (15 seconds). */

  /**
   * @brief Updates the last receive time to the current time.
   */
  void updateLastReceiveTime();

  /**
   * @brief Called when data is received over I2C.
   *
   * @param howMany Number of bytes received.
   */
  static void receiveEvent(int howMany);

  /**
   * @brief Called when the master requests data from the I2C slave.
   */
  static void requestEvent();

  /**
   * @brief The task that handles I2C communication in the background.
   *
   * @param parameter Task parameter (unused).
   */
  static void task(void *parameter);
};

#endif
