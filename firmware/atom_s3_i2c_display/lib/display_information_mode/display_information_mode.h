#ifndef ATOM_S3_DISPLAY_INFORMATION_MODE_H
#define ATOM_S3_DISPLAY_INFORMATION_MODE_H

#include <communication_base.h>
#include <mode.h>
#include <primitive_lcd.h>

class DisplayInformationMode : public Mode {
public:
    DisplayInformationMode();

private:
    void task(PrimitiveLCD &lcd, CommunicationBase &com) override;
};

#endif  // DISPLAY_INFORMATION_MODE_H
