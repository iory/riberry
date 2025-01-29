#ifndef ATOM_S3_SERVO_CONTROL_MODE_H
#define ATOM_S3_SERVO_CONTROL_MODE_H

#include <mode.h>
#include <primitive_lcd.h>
#include <communication_base.h>

class ServoControlMode : public Mode {
public:
  ServoControlMode();

private:
  void task(PrimitiveLCD &lcd, CommunicationBase &com);
};

#endif // ATOM_S3_SERVO_CONTROL_MODE_H
