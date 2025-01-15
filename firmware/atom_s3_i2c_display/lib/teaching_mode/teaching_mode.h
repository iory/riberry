#ifndef ATOM_S3_TEACHING_MODE_H
#define ATOM_S3_TEACHING_MODE_H

#include <mode.h>
#include <primitive_lcd.h>
#include <communication_base.h>

class TeachingMode : public Mode {
public:
  TeachingMode(PrimitiveLCD &lcd, CommunicationBase &i2c);
  void createTask(uint8_t xCoreID) override;

private:
  static TeachingMode* instance; /**< Singleton instance of TeachingMode. */
  PrimitiveLCD &lcd;
  CommunicationBase &comm;

  static void task(void *parameter);
  void drawARMarker(int marker_id, int x, int y, int size);
};

#endif // ATOM_S3_TEACHING_MODE_H
