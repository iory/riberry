#ifndef ATOM_S3_PRESSURE_CONTROL_MODE_H
#define ATOM_S3_PRESSURE_CONTROL_MODE_H

#include <mode.h>
#include <primitive_lcd.h>
#include <communication_base.h>

class PressureControlMode : public Mode {
public:
  PressureControlMode(PrimitiveLCD &lcd, CommunicationBase &i2c);
  void createTask(uint8_t xCoreID) override;

private:
  static PressureControlMode* instance; /**< Singleton instance of PressureControlMode. */
  PrimitiveLCD &lcd;
  CommunicationBase &comm;

  static void task(void *parameter);
};

#endif // ATOM_S3_PRESSURE_CONTROL_MODE_H
