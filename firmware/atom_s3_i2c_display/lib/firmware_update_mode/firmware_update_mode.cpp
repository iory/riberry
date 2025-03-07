#include <firmware_update_mode.h>

FirmwareUpdateMode::FirmwareUpdateMode() : Mode(ModeType::FIRMWARE_UPDATE, true) {}

void FirmwareUpdateMode::task(PrimitiveLCD &lcd, CommunicationBase &com) {
    while (running) {
        lcd.drawBlack();
        lcd.printColorText("Waiting for firmware update...");
        delayWithTimeTracking(pdMS_TO_TICKS(1000));
    }
}
