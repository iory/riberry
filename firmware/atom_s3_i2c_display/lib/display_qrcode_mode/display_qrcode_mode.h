#ifndef DISPLAY_QRCODE_MODE_H
#define DISPLAY_QRCODE_MODE_H

#include <mode.h>
#include <primitive_lcd.h>
#include <communication_base.h>

class DisplayQRcodeMode : public Mode {
public:
  DisplayQRcodeMode(PrimitiveLCD &lcd, CommunicationBase &i2c);
  void createTask(uint8_t xCoreID) override;

private:
  static DisplayQRcodeMode* instance; /**< Singleton instance of DisplayQRcodeMode. */
  PrimitiveLCD &lcd;
  CommunicationBase &comm;

  static void task(void *parameter);
};

#endif // DISPLAY_QRCODE_MODE_H
