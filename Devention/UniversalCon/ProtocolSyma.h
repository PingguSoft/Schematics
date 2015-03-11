#ifndef ProtocolSyma_h
#define ProtocolSyma_h

#include "Protocol.h"
#include "Timer.h"

// For code readability
enum {
    CHANNEL1 = 0,
    CHANNEL2,
    CHANNEL3,
    CHANNEL4,
    CHANNEL5,
    CHANNEL6,
    CHANNEL7,
    CHANNEL8,
    CHANNEL9,
    CHANNEL10
};

// Packet ack status values
enum {
    PKT_PENDING = 0,
    PKT_ACKED,
    PKT_TIMEOUT
};

#define FORMAT_YD717        0
#define FORMAT_SKYWLKR      1
#define FORMAT_XINXUN       2

// Bit vector from bit position
#define BV(bit) (1 << bit)

class ProtocolSyma : public Protocol
{

public:
    ProtocolSyma():Protocol(TX_MOD_NRF24L01, PROTO_YD717){};
    Timer getTimer();

    virtual void update();
    virtual int  init(u32 id);
    virtual int  deinit();
    virtual int  reset();
    virtual int  bind(u32 id);
    virtual int  getChannels();
    virtual int  setPower(int power);

private:

protected:

};

#endif
