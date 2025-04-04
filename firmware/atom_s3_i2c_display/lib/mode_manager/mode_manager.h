#ifndef MODE_MANAGER_H
#define MODE_MANAGER_H

#include <button_manager.h>
#include <communication_base.h>
#include <mode.h>
#include <primitive_lcd.h>

#include <vector>

#include "execution_timer.h"

class ModeManager : public ExecutionTimer {
public:
    ModeManager(PrimitiveLCD &lcd,
                ButtonManager &button,
                CommunicationBase &i2c,
                const std::vector<Mode *> &modes);
    void createTask(uint8_t xCoreID);
    void startCurrentMode();
    void addSelectedMode(Mode &mode);

private:
    static ModeManager *instance;
    ButtonManager &button_manager;
    PrimitiveLCD &lcd;
    CommunicationBase &comm;

    static const std::vector<Mode *> *allModes;
    static std::vector<Mode *> selectedModes;
    static uint8_t selectedModesBytes[100];
    static int current_mode_index;

    static void task(void *parameter);
    bool isValidIndex(const std::vector<Mode *> &vec, int index);
    void changeMode(int suspend_mode_index, int resume_mode_index);
    void switchToNextMode();
    void stopCurrentMode();
    void deleteSelectedModes();
    void suspendSelectedModes();
    void setRequestBytes();
};

#endif
