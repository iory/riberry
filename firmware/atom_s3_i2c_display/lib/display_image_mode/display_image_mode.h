#ifndef DISPLAY_IMAGE_MODE_H
#define DISPLAY_IMAGE_MODE_H

#include <mode.h>
#include <primitive_lcd.h>
#include <communication_base.h>

class DisplayImageMode : public Mode {
public:
  DisplayImageMode(PrimitiveLCD &lcd, CommunicationBase &i2c);
  void createTask(uint8_t xCoreID) override;

private:
  static DisplayImageMode* instance; /**< Singleton instance of DisplayImageMode. */
  PrimitiveLCD &lcd;
  CommunicationBase &comm;

  static void task(void *parameter);
};

#endif // DISPLAY_IMAGE_MODE_H
