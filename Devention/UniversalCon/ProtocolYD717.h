#ifndef ProtocolYD717_h
#define ProtocolYD717_h

#include "Protocol.h"

#define BIND_COUNT 60
#define dbgprintf if(0)  printf
#define PACKET_PERIOD    8000     // Timeout for callback in uSec, 8ms=8000us for YD717
#define INITIAL_WAIT    50000     // Initial wait before starting callbacks
#define PACKET_CHKTIME    500     // Time to wait if packet not yet acknowledged or timed out    

// Stock tx fixed frequency is 0x3C. Receiver only binds on this freq.
#define RF_CHANNEL      0x3C
#define FLAG_FLIP       0x0F
#define FLAG_LIGHT      0x10

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

enum {
    YD717_INIT1 = 0,
    YD717_BIND2,
    YD717_BIND3,
    YD717_DATA
};

// Packet ack status values
enum {
    PKT_PENDING = 0,
    PKT_ACKED,
    PKT_TIMEOUT
};

#define PAYLOADSIZE         8   // receive data pipes set to this size, but unused
#define MAX_PACKET_SIZE     9   // YD717 packets have 8-byte payload, Syma X4 is 9

#define FORMAT_YD717        0
#define FORMAT_SKYWLKR      1
#define FORMAT_XINXUN       2

// Bit vector from bit position
#define BV(bit) (1 << bit)

class ProtocolYD717 : public Protocol
{

public:
    ProtocolYD717():Protocol(TX_MOD_NRF24L01, PROTO_YD717){};

private:
    virtual int init(u32 id) = 0;
    virtual int deinit() = 0;
    virtual int reset() = 0;
    virtual int bind(u32 id) = 0;
    virtual int getChannels() = 0;
    virtual int setPower(int power) = 0;
    
    u8   ackPacket();
    void sendPacket(u8 bind);
    void init1();
    void init2();
    void init3();
    void updateTelemetry();
    u16  Callback();
    void TimerCallback();
    void setRxTxAddr(u32 id);
    
    
    
    u8  packet[MAX_PACKET_SIZE];
    u16 counter;
    u32 packet_counter;
    u8  tx_power, flags;
    u8  rx_tx_addr[5];
    u8  phase;
};

#endif
