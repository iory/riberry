#ifndef ATOM_S3_DISPLAY_INFORMATION_MODE_H
#define ATOM_S3_DISPLAY_INFORMATION_MODE_H

#include <mode.h>
#include <atom_s3_lcd.h>
#include <communication_base.h>

class DisplayInformationMode : public Mode {
public:
  DisplayInformationMode(AtomS3LCD &lcd, CommunicationBase &i2c);
  void createTask(uint8_t xCoreID) override;

private:
  static DisplayInformationMode* instance; /**< Singleton instance of DisplayInformationMode. */
  AtomS3LCD &atoms3lcd;
  CommunicationBase &comm;

  static void task(void *parameter);
};

#endif // DISPLAY_INFORMATION_MODE_H
