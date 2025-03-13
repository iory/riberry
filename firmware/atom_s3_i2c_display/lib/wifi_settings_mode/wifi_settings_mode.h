#ifndef WIFI_SETTINGS_MODE_H
#define WIFI_SETTINGS_MODE_H

#include <button_manager.h>
#include <communication_base.h>
#include <mode.h>
#include <primitive_lcd.h>

class WiFiSettingsMode : public Mode {
public:
    WiFiSettingsMode(ButtonManager &button_manager);

private:
    void task(PrimitiveLCD &lcd, CommunicationBase &com) override;

    ButtonManager &button_manager;
};

#endif  // WIFI_SETTINGS_MODE_H
