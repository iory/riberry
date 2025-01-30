#ifndef ATOM_S3_TEACHING_MODE_H
#define ATOM_S3_TEACHING_MODE_H

#include <communication_base.h>
#include <mode.h>
#include <primitive_lcd.h>

class TeachingMode : public Mode {
public:
    TeachingMode();

private:
    void task(PrimitiveLCD &lcd, CommunicationBase &com) override;
    void drawARMarker(int marker_id, int x, int y, int size, PrimitiveLCD &lcd);
};

#endif  // TEACHING_MODE_H
