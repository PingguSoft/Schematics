#ifndef ProtocolSyma_h
#define ProtocolSyma_h

#include "DeviceNRF24L01.h"
#include "Protocol.h"
#include "Timer.h"

class ProtocolSyma : public Protocol, public Timer
{
#define PAYLOADSIZE         10  // receive data pipes set to this size, but unused
#define MAX_PACKET_SIZE     16  // X11,X12,X5C-1 10-byte, X5C 16-byte
#define MAX_BIND_COUNT     345

#define PACKET_PERIOD_MS     4
#define INITIAL_WAIT_MS      1
#define FIRST_PACKET_MS     12

#define FLAG_FLIP         0x01
#define FLAG_VIDEO        0x02
#define FLAG_PICTURE      0x04

#define MAX_RF_CHANNELS     17

enum {
    SYMAX_INIT1 = 0,
    SYMAX_BIND2,
    SYMAX_BIND3,
    SYMAX_DATA
};

enum {
    FORMAT_OTHERS  = 0,
    FORMAT_X5C_X2  = 1,
};

public:
    ProtocolSyma():Protocol(TX_MOD_NRF24L01, PROTO_SymaX) { }
    ~ProtocolSyma() { close(); }
    
// for timer
    virtual void handleTimer(s8 id);

// for protocol
    virtual void loop(void);
    virtual int  init(void);
    virtual int  close(void);
    virtual int  reset(void);
    virtual int  getChannels(void);
    virtual int  setPower(int power);
    virtual void test(s8 id);

private:
    u8   getCheckSum(u8 *data);
    u8   checkStatus(void);
    u8   getChannel(CH_T id);
    void buildPacketX5C(u8 bind);
    void buildPacket(u8 bind);
    void sendPacket(u8 bind);
    void initRxTxAddr(void);
    void init1(void);
    void init2(void);
    void init3(void);
    void setRFChannel(u8 address);
    u16  callState(void);
    
    void testUp(void);
    void testDown(void);

// variables
    DeviceNRF24L01  mDev;
    u8   mCurChan;
    u8   mChannelBuf[MAX_RF_CHANNELS];
    u8   mChannelCnt;

    u8   mProtoOpt;
    u8   mPacketBuf[MAX_PACKET_SIZE];
    u8   mPacketSize;
    u16  mBindCtr;
    u32  mPacketCtr;
    u8   mAuxFlag;
    u8   mRxTxAddrBuf[5];
    u8   mState;

    s8   mTmrState;
    s8   mTmrTest;
    s8   mTmrTest2;
    u8   mMode;
    s32  mThr;

protected:

};

#endif
