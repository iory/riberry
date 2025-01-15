#ifndef ATOM_S3_SERVO_CONTROL_MODE_H
#define ATOM_S3_SERVO_CONTROL_MODE_H

#include <mode.h>
#include <primitive_lcd.h>
#include <communication_base.h>

class ServoControlMode : public Mode {
public:
  ServoControlMode(PrimitiveLCD &lcd, CommunicationBase &i2c);
  void createTask(uint8_t xCoreID) override;

private:
  static ServoControlMode* instance; /**< Singleton instance of ServoControlMode. */
  PrimitiveLCD &lcd;
  CommunicationBase &comm;

  static void task(void *parameter);
};

#endif // ATOM_S3_SERVO_CONTROL_MODE_H
