#ifndef ATOM_S3_DISPLAY_BATTERY_MODE3_H
#define ATOM_S3_DISPLAY_BATTERY_MODE3_H

#include <communication_base.h>
#include <mode.h>
#include <primitive_lcd.h>

class DisplayOdomMode : public Mode {
public:
    DisplayOdomMode();

private:
    void task(PrimitiveLCD &lcd, CommunicationBase &com);
};

#endif  // DISPLAY_INFORMATION_MODE_H
