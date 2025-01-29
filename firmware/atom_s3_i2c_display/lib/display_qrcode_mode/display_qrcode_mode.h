#ifndef DISPLAY_QRCODE_MODE_H
#define DISPLAY_QRCODE_MODE_H

#include <mode.h>
#include <primitive_lcd.h>
#include <communication_base.h>

class DisplayQRcodeMode : public Mode {
public:
  DisplayQRcodeMode();

private:
  void task(PrimitiveLCD &lcd, CommunicationBase &com) override;
};

#endif // DISPLAY_QRCODE_MODE_H
