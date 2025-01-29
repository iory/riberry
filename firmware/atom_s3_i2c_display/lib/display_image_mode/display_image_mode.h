#ifndef DISPLAY_IMAGE_MODE_H
#define DISPLAY_IMAGE_MODE_H

#include <communication_base.h>
#include <mode.h>
#include <primitive_lcd.h>

class DisplayImageMode : public Mode {
public:
    DisplayImageMode();

private:
    void task(PrimitiveLCD &lcd, CommunicationBase &com) override;
};

#endif  // DISPLAY_IMAGE_MODE_H
