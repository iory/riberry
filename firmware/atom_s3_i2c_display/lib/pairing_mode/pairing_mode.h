#ifndef PAIRING_MODE_H
#define PAIRING_MODE_H

#include <button_manager.h>
#include <communication_base.h>
#include <mode.h>
#include <pairing.h>
#include <primitive_lcd.h>

class PairingMode : public Mode {
public:
    PairingMode(ButtonManager &button_manager, Pairing &pairing);
    void suspendTask() override;
    void resumeTask() override;

private:
    void task(PrimitiveLCD &lcd, CommunicationBase &com) override;

    ButtonManager &button_manager;
    Pairing &pairing;
};

#endif  // PAIRING_MODE_H
