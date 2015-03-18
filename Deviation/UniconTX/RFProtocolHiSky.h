#ifndef _PROTOCOL_HISKY_
#define _PROTOCOL_HISKY_

#include "DeviceNRF24L01.h"
#include "RFProtocol.h"

class RFProtocolHiSky : public RFProtocol
{

#define MAX_PACKET_SIZE     10
#define MAX_BIND_COUNT     800

#define PACKET_PERIOD_MS     1
#define PACKET_CHKTIME       1
#define INITIAL_WAIT_MS      1
#define ADDR_BUF_SIZE        5
#define MAX_RF_CHANNELS     20


enum {
    HISKY_INIT = 0,
    HISKY_DATA = 0x10
};

public:
    RFProtocolHiSky():RFProtocol(RFProtocol::TX_NRF24L01, RFProtocol::PROTO_NRF24L01_HISKY) { }
    RFProtocolHiSky(u32 id):RFProtocol(id) { }
    ~RFProtocolHiSky() { close(); }

// for protocol
    virtual void loop(void);
    virtual int  init(void);
    virtual int  close(void);
    virtual int  reset(void);
    virtual int  getChannels(void);
    virtual int  getInfo(s8 id, u8 *data);
    virtual void test(s8 id);
    virtual u16  callState(void);

private:
    void buildRFChannels(u32 seed);
    void buildBindingPacket(void);
    void buildDataPacket(void);
    void initRxTxAddr(void);
    void init1(void);
    void setTxID(u32 id);
    u16  getChannel(CH_T id);


// variables
    DeviceNRF24L01  mDev;
    u8   mCurChan;
    u8   mChannelBuf[MAX_RF_CHANNELS];
    u8   mPacketBuf[MAX_PACKET_SIZE];

    u8   mBindingIdx;
    u8   mBindingBufs[4][MAX_PACKET_SIZE];
    u8   mCtr1ms;
    
    u16  mBindCtr;
    u32  mPacketCtr;
    u8   mRxTxAddrBuf[ADDR_BUF_SIZE];
    u8   mState;
protected:

};

#endif
