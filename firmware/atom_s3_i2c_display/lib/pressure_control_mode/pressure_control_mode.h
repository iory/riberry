#ifndef ATOM_S3_PRESSURE_CONTROL_MODE_H
#define ATOM_S3_PRESSURE_CONTROL_MODE_H

#include <mode.h>
#include <atom_s3_lcd.h>
#include <atom_s3_i2c.h>

class PressureControlMode : public Mode {
public:
  PressureControlMode(AtomS3LCD &lcd, AtomS3I2C &i2c);
  void createTask(uint8_t xCoreID) override;

private:
  static PressureControlMode* instance; /**< Singleton instance of PressureControlMode. */
  AtomS3LCD &atoms3lcd;
  AtomS3I2C &atoms3i2c;

  static void task(void *parameter);
};

#endif // ATOM_S3_PRESSURE_CONTROL_MODE_H
