#ifndef ATOM_S3_DISPLAY_INFORMATION_MODE_H
#define ATOM_S3_DISPLAY_INFORMATION_MODE_H

#include <mode.h>
#include <atom_s3_lcd.h>
#include <atom_s3_i2c.h>

class DisplayInformationMode : public Mode {
public:
  DisplayInformationMode(AtomS3LCD &lcd, AtomS3I2C &i2c);
  void createTask(uint8_t xCoreID) override;

private:
  static DisplayInformationMode* instance; /**< Singleton instance of DisplayInformationMode. */
  AtomS3LCD &atoms3lcd;
  AtomS3I2C &atoms3i2c;

  static void task(void *parameter);
};

#endif // DISPLAY_INFORMATION_MODE_H
