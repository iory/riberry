#include <pressure_control_mode.h>

PressureControlMode* PressureControlMode::instance = nullptr;

PressureControlMode::PressureControlMode(PrimitiveLCD &lcd, CommunicationBase &i2c)
  : lcd(lcd), comm(i2c), Mode("PressureControlMode") {
    instance = this;
}

void PressureControlMode::task(void *parameter) {
  while (true) {
    instance->comm.setRequestStr(instance->getModeName());
    // Check for I2C timeout
    if (instance->comm.checkTimeout()) {
      instance->lcd.drawNoDataReceived();
      instance->lcd.printColorText(instance->getModeName() + "\n");
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    // Display information
    else {
      instance->lcd.drawBlack();
      if (instance->lcd.color_str.isEmpty())
        instance->lcd.printColorText("Waiting for " + instance->getModeName());
      else
        instance->lcd.printColorText(instance->lcd.color_str);
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
}

void PressureControlMode::createTask(uint8_t xCoreID) {
  xTaskCreatePinnedToCore(task, "Pressure Control Mode", 2048, NULL, 1, &taskHandle, xCoreID);
}
