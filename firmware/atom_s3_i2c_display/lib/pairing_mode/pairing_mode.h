#ifndef PAIRING_MODE_H
#define PAIRING_MODE_H

#include <mode.h>
#include <primitive_lcd.h>
#include <communication_base.h>

#ifdef SENDER
#include <pairing_sender.h>
#else
#include <pairing_receiver.h>
#endif

class PairingMode : public Mode {
public:
  PairingMode(PrimitiveLCD &lcd, CommunicationBase &com);
  void createTask(uint8_t xCoreID) override;

private:
  static PairingMode* instance; /**< Singleton instance of PairingMode. */
  PrimitiveLCD &lcd;
  CommunicationBase &comm;

  static void task(void *parameter);

#ifdef SENDER
  PairingSender pairing_com;
#else
  PairingReceiver pairing_com;
#endif

};

#endif // PAIRING_MODE_H
