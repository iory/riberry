#ifndef ATOM_S3_MODE_MANAGER_H
#define ATOM_S3_MODE_MANAGER_H

#include <vector>

#include <button_manager.h>
#include <atom_s3_lcd.h>
#include <communication_base.h>

#include <mode.h>

class AtomS3ModeManager {
public:
  AtomS3ModeManager(AtomS3LCD &lcd, ButtonManager &button, CommunicationBase &i2c, const std::vector<Mode *> &modes);
  void createTask(uint8_t xCoreID);
  void initializeSelectedModes();
  void startCurrentMode();
  void addSelectedMode(Mode &mode);

private:
  static AtomS3ModeManager* instance; /**< Singleton instance of AtomS3ModeManager. */
  // Common tasks for AtomS3
  ButtonManager &button_manager;
  AtomS3LCD &atoms3lcd;
  CommunicationBase &comm;

  static const std::vector<Mode*> *allModes;
  static std::vector<Mode*> selectedModes;
  static String selectedModesStr;
  static int current_mode_index;

  static void task(void *parameter);
  bool isValidIndex(const std::vector<Mode*>& vec, int index);
  void changeMode(int suspend_mode_index, int resume_mode_index);
  void stopCurrentMode();
  void deleteSelectedModes();
};

#endif
