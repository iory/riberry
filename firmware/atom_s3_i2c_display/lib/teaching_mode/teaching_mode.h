#ifndef ATOM_S3_TEACHING_MODE_H
#define ATOM_S3_TEACHING_MODE_H

#include <mode.h>
#include <primitive_lcd.h>
#include <communication_base.h>

class TeachingMode : public Mode {
public:
  TeachingMode();

private:
  void task(PrimitiveLCD &lcd, CommunicationBase &com) override;
  void drawARMarker(int marker_id, int x, int y, int size, PrimitiveLCD &lcd);
};

#endif // ATOM_S3_TEACHING_MODE_H
