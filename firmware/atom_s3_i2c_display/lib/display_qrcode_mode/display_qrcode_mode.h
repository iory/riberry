#ifndef DISPLAY_QRCODE_MODE_H
#define DISPLAY_QRCODE_MODE_H

#include <communication_base.h>
#include <mode.h>
#include <primitive_lcd.h>

class DisplayQRcodeMode : public Mode {
public:
    DisplayQRcodeMode(bool isSkippable = false);

private:
    void task(PrimitiveLCD &lcd, CommunicationBase &com) override;
};

#endif  // DISPLAY_QRCODE_MODE_H
