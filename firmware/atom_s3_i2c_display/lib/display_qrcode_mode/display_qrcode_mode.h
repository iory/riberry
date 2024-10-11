#ifndef DISPLAY_QRCODE_MODE_H
#define DISPLAY_QRCODE_MODE_H

#include <mode.h>
#include <atom_s3_lcd.h>
#include <atom_s3_i2c.h>

class DisplayQRcodeMode : public Mode {
public:
  DisplayQRcodeMode(AtomS3LCD &lcd, AtomS3I2C &i2c);
  void createTask(uint8_t xCoreID) override;

private:
  static DisplayQRcodeMode* instance; /**< Singleton instance of DisplayQRcodeMode. */
  AtomS3LCD &atoms3lcd;
  AtomS3I2C &atoms3i2c;

  static void task(void *parameter);
};

#endif // DISPLAY_QRCODE_MODE_H
