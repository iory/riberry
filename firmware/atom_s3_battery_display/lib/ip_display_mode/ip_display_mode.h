#ifndef ATOM_S3_DISPLAY_BATTERY_MODE2_H
#define ATOM_S3_DISPLAY_BATTERY_MODE2_H

#include <mode.h>
#include <atom_s3_lcd.h>
#include <atom_s3_i2c.h>

class IpDisplayMode : public Mode {
public:
  IpDisplayMode(AtomS3LCD &lcd, AtomS3I2C &i2c);
  void createTask(uint8_t xCoreID) override;

private:
  static IpDisplayMode* instance; /**< Singleton instance of DisplayInformationMode. */
  AtomS3LCD &atoms3lcd;
  AtomS3I2C &atoms3i2c;

  static void task(void *parameter);
};

#endif // DISPLAY_INFORMATION_MODE_H
