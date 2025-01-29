#ifndef ATOM_S3_SERVO_CONTROL_MODE_H
#define ATOM_S3_SERVO_CONTROL_MODE_H

#include <communication_base.h>
#include <mode.h>
#include <primitive_lcd.h>

class ServoControlMode : public Mode {
public:
    ServoControlMode();

private:
    void task(PrimitiveLCD &lcd, CommunicationBase &com);
};

#endif  // ATOM_S3_SERVO_CONTROL_MODE_H
