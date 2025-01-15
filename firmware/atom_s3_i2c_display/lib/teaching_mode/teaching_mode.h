#ifndef ATOM_S3_TEACHING_MODE_H
#define ATOM_S3_TEACHING_MODE_H

#include <mode.h>
#include <atom_s3_lcd.h>
#include <communication_base.h>

class TeachingMode : public Mode {
public:
  TeachingMode(AtomS3LCD &lcd, CommunicationBase &i2c);
  void createTask(uint8_t xCoreID) override;

private:
  static TeachingMode* instance; /**< Singleton instance of TeachingMode. */
  AtomS3LCD &atoms3lcd;
  CommunicationBase &comm;

  static void task(void *parameter);
  void drawARMarker(int marker_id, int x, int y, int size);
};

#endif // ATOM_S3_TEACHING_MODE_H
