#ifndef PRESSURE_CONTROL_MODE_H
#define PRESSURE_CONTROL_MODE_H

#include <mode.h>

class PressureControlMode : public Mode {
public:
    PressureControlMode() : Mode("PressureControlMode") {}
};

#endif  // PRESSURE_CONTROL_MODE_H
