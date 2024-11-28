#ifndef ATOM_S3_DISPLAY_BATTERY_MODE3_H
#define ATOM_S3_DISPLAY_BATTERY_MODE3_H

#include <mode.h>
#include <atom_s3_lcd.h>
#include <atom_s3_i2c.h>

class DisplayOdomMode : public Mode {
public:
  DisplayOdomMode(AtomS3LCD &lcd, AtomS3I2C &i2c);
  void createTask(uint8_t xCoreID) override;

private:
  static DisplayOdomMode* instance; /**< Singleton instance of DisplayInformationMode. */
  AtomS3LCD &atoms3lcd;
  AtomS3I2C &atoms3i2c;

  static void task(void *parameter);
};

#endif // DISPLAY_INFORMATION_MODE_H
