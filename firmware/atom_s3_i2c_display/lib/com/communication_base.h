#ifndef COMMUNICATION_BASE_H
#define COMMUNICATION_BASE_H

#include <SerialTransfer.h>
#include <WireSlave.h>
#include <button_manager.h>
#include <primitive_lcd.h>

#include "execution_timer.h"
#include "packet.h"
#include "pairing.h"
#include "role.h"

class CommunicationBase : public ExecutionTimer {
public:
    virtual ~CommunicationBase() = default;

    CommunicationBase(PrimitiveLCD& lcd,
                      ButtonManager& button,
                      Pairing& pairing,
                      Stream* stream,
                      Role role = Role::Main);

    void createTask(uint8_t xCoreID);

    bool checkTimeout();

    int splitString(const String& input, char delimiter, char* output[], int maxParts);

    void stopReceiveEvent();
    void startReceiveEvent();
    static void setRequestBytes(uint8_t* bytes, size_t byteLen);

    static uint8_t forcedMode;
    static uint8_t selectedModesBytes[100];
    static Role role;
    void startPairing() { pairingEnabled = true; }
    void stopPairing() { pairingEnabled = false; }

    static void setStream(Stream* stream);
    Stream* getStream() const;

    template <typename T>
    static void write(const T& data, const size_t len);

    static SerialTransfer transfer;

    static void stopStream() { _stopStream = true; }
    static void startStream() { _stopStream = false; }

private:
    static Stream* _stream;

    static uint8_t buffer[256];
    static bool _stopStream;

    bool receiveEventEnabled;
    static CommunicationBase* instance;
    PrimitiveLCD& lcd;
    ButtonManager& button_manager;
    Pairing& pairing;
    static bool pairingEnabled;

    unsigned long lastReceiveTime = 0;
    const unsigned long receiveTimeout = 15000;
    static const unsigned long PACKET_TIMEOUT = 200;

    static constexpr uint8_t jpegPacketHeader[3] = {0xFF, 0xD8, 0xEA};
    static constexpr uint8_t qrCodeHeader = 0x02;
    static constexpr uint8_t forceModeHeader[3] = {0xFF, 0xFE, 0xFD};
    static constexpr uint8_t selectedModesHeader[3] = {0xFC, 0xFB, 0xFA};

    // Limited to a maximum of 100 bytes to prevent buffer overflow
    static uint8_t requestBytes[100]; /**< Bytes to be sent on I2C request. */

    void updateLastReceiveTime();

    static void receiveEvent(int howMany);
    static void handleJpegPacket(const String& str);
    static void handleQrCodePacket(const String& str, int offset);
    static void handleForceModePacket(const String& str, int offset);
    static void handleSelectedModePacket(const String& str, int offset);
    static void requestEvent();
    static void processPacket(const String& str, int offset);
    static void task(void* parameter);
};

#endif
