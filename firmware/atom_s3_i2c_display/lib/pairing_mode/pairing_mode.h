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
  PairingMode();

private:
  void task(PrimitiveLCD &lcd, CommunicationBase &com) override;

#ifdef SENDER
  PairingSender _pairing_com;
#else
  PairingReceiver _pairing_com;
#endif

};

#endif // PAIRING_MODE_H
