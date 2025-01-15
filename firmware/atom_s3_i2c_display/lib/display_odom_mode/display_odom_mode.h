#ifndef ATOM_S3_DISPLAY_BATTERY_MODE3_H
#define ATOM_S3_DISPLAY_BATTERY_MODE3_H

#include <mode.h>
#include <primitive_lcd.h>
#include <communication_base.h>

class DisplayOdomMode : public Mode {
public:
  DisplayOdomMode(PrimitiveLCD &lcd, CommunicationBase &i2c);
  void createTask(uint8_t xCoreID) override;

private:
  static DisplayOdomMode* instance; /**< Singleton instance of DisplayInformationMode. */
  PrimitiveLCD &lcd;
  CommunicationBase &comm;

  static void task(void *parameter);
};

#endif // DISPLAY_INFORMATION_MODE_H
