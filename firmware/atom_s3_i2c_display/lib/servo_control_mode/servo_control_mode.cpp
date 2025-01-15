#include <servo_control_mode.h>

ServoControlMode* ServoControlMode::instance = nullptr;

ServoControlMode::ServoControlMode(PrimitiveLCD &lcd, CommunicationBase &i2c)
  : lcd(lcd), comm(i2c), Mode("ServoControlMode") {
    instance = this;
}

void ServoControlMode::task(void *parameter) {
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

void ServoControlMode::createTask(uint8_t xCoreID) {
  xTaskCreatePinnedToCore(task, "Servo Control Mode", 2048, NULL, 1, &taskHandle, xCoreID);
}
