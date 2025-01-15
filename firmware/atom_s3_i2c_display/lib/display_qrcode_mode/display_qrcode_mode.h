#ifndef DISPLAY_QRCODE_MODE_H
#define DISPLAY_QRCODE_MODE_H

#include <mode.h>
#include <atom_s3_lcd.h>
#include <communication_base.h>

class DisplayQRcodeMode : public Mode {
public:
  DisplayQRcodeMode(AtomS3LCD &lcd, CommunicationBase &i2c);
  void createTask(uint8_t xCoreID) override;

private:
  static DisplayQRcodeMode* instance; /**< Singleton instance of DisplayQRcodeMode. */
  AtomS3LCD &atoms3lcd;
  CommunicationBase &comm;

  static void task(void *parameter);
};

#endif // DISPLAY_QRCODE_MODE_H
