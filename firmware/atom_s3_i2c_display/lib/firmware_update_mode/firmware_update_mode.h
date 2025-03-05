#ifndef FIRMWARE_UPDATE_MODE_H
#define FIRMWARE_UPDATE_MODE_H

#include <communication_base.h>
#include <mode.h>
#include <primitive_lcd.h>

class FirmwareUpdateMode : public Mode {
public:
    FirmwareUpdateMode();

private:
    void task(PrimitiveLCD &lcd, CommunicationBase &com) override;
};

#endif  // FIRMWARE_UPDATE_MODE_H
