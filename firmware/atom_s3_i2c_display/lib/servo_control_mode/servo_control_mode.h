#ifndef SERVO_CONTROL_MODE_H
#define SERVO_CONTROL_MODE_H

#include <mode.h>

class ServoControlMode : public Mode {
public:
    ServoControlMode() : Mode("ServoControlMode") {}
};

#endif  // SERVO_CONTROL_MODE_H
