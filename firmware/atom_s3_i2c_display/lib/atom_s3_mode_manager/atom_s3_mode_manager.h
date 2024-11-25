#ifndef ATOM_S3_MODE_MANAGER_H
#define ATOM_S3_MODE_MANAGER_H

#include <vector>

#include <atom_s3_button.h>
#include <atom_s3_lcd.h>
#include <atom_s3_i2c.h>

#include <mode.h>

class AtomS3ModeManager {
public:
  AtomS3ModeManager(AtomS3LCD &lcd, AtomS3Button &button, AtomS3I2C &i2c);
  void createTask(uint8_t xCoreID);
  void initializeAllModes(uint8_t xCoreID);
  void startCurrentMode();
  void addMode(Mode &mode);

private:
  static AtomS3ModeManager* instance; /**< Singleton instance of AtomS3ModeManager. */
  // Common tasks for AtomS3
  AtomS3Button &atoms3button;
  AtomS3LCD &atoms3lcd;
  AtomS3I2C &atoms3i2c;

  static std::vector<Mode*> allModes;
  static int current_mode_index;

  static void task(void *parameter);
  void changeMode(int suspend_mode_index, int resume_mode_index);
};

#endif
