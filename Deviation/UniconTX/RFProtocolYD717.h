#ifndef _PROTOCOL_YD717_
#define _PROTOCOL_YD717_

#include "DeviceNRF24L01.h"
#include "RFProtocol.h"
#include "Timer.h"


class RFProtocolYD717 : public RFProtocol, public Timer
{
#define PAYLOADSIZE          8  // receive data pipes set to this size, but unused
#define MAX_PACKET_SIZE      9  // YD717 packets have 8-byte payload, Syma X4 is 9
#define MAX_BIND_COUNT      60

#define PACKET_PERIOD_MS     8
#define INITIAL_WAIT_MS     50
#define PACKET_CHKTIME_MS    5  // Time to wait if packet not yet acknowledged or timed out

// Stock tx fixed frequency is 0x3C. Receiver only binds on this freq.
#define RF_CHANNEL          0x3C
#define FLAG_FLIP           0x0F
#define FLAG_LIGHT          0x10

// Packet ack status values
enum {
    PKT_PENDING = 0,
    PKT_ACKED,
    PKT_TIMEOUT
};

enum {
    YD717_INIT1 = 0,
    YD717_BIND2,
    YD717_BIND3,
    YD717_DATA
};

enum {
    FORMAT_YD717       = 0,
    FORMAT_SKY_WALKER  = 1,
    FORMAT_XINXUN      = 2,
    FORMAT_NI_HUI      = 3,
    FORMAT_SYMA_X4     = 4,
};

public:
    RFProtocolYD717():RFProtocol(RFProtocol::TX_NRF24L01, RFProtocol::PROTO_NRF24L01_YD717) { }
    RFProtocolYD717(u32 id):RFProtocol(id) { }
    ~RFProtocolYD717() { close(); }

// for timer
    virtual void handleTimer(s8 id);

// for protocol
    virtual void loop(void);
    virtual int  init(void);
    virtual int  close(void);
    virtual int  reset(void);
    virtual int  getChannels(void);
    virtual int  setPower(int power);
    virtual int  getInfo(s8 id, u8 *data, u8 *size);
    virtual void test(s8 id);

private:
    u8   getCheckSum(u8 *data);
    u8   checkStatus(void);
    u8   getControl(CH_T id);
    void getControls(u8* throttle, u8* rudder, u8* elevator, u8* aileron,
                     u8* flags, u8* rudder_trim, u8* elevator_trim, u8* aileron_trim);
    void sendPacket(u8 bind);
    void initRxTxAddr(void);
    void init1(void);
    void init2(void);
    void init3(void);
    void setRFChannel(u8 address);
    u16  callState(void);
    void updateTelemetry(void);
    
    void testUp(void);
    void testDown(void);

// variables
    DeviceNRF24L01  mDev;

    u8   mProtoOpt;
    u8   mPacketBuf[MAX_PACKET_SIZE];
    u8   mPacketSize;
    u16  mBindCtr;
    u32  mPacketCtr;
    u8   mAuxFlag;
    u8   mRxTxAddrBuf[5];
    u8   mState;
    s8   mTmrState;

protected:

};

#endif
