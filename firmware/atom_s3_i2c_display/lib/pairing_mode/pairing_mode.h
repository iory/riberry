#ifndef PAIRING_MODE_H
#define PAIRING_MODE_H

#include <Preferences.h>
#include <button_manager.h>
#include <communication_base.h>
#include <mode.h>
#include <pairing.h>
#include <primitive_lcd.h>

class PairingMode : public Mode {
public:
    PairingMode(ButtonManager &button_manager, Pairing &pairing, CommunicationBase &com);
    void suspendTask() override;
    void resumeTask() override;

private:
    void task(PrimitiveLCD &lcd, CommunicationBase &com) override;

    ButtonManager &button_manager;
    CommunicationBase &com;
    Pairing &pairing;
    Preferences preferences;
};

#endif  // PAIRING_MODE_H
