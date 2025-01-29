#ifndef ATOM_S3_PRESSURE_CONTROL_MODE_H
#define ATOM_S3_PRESSURE_CONTROL_MODE_H

#include <communication_base.h>
#include <mode.h>
#include <primitive_lcd.h>

class PressureControlMode : public Mode {
public:
    PressureControlMode();

private:
    void task(PrimitiveLCD &lcd, CommunicationBase &com) override;
};

#endif  // ATOM_S3_PRESSURE_CONTROL_MODE_H
