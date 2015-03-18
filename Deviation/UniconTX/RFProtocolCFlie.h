#ifndef _PROTOCOL_CFLIE_
#define _PROTOCOL_CFLIE_

#include "DeviceNRF24L01.h"
#include "RFProtocol.h"

class RFProtocolCFlie : public RFProtocol
{
#define PAYLOADSIZE          8
#define MAX_PACKET_SIZE     15
#define MAX_BIND_COUNT      60

#define PACKET_PERIOD_MS    10
#define PACKET_CHK_MS        1
#define INITIAL_WAIT_MS     50

#define ADDR_BUF_SIZE        5
#define MAX_RF_CHANNELS     20


// Packet ack status values
enum {
    PKT_PENDING = 0,
    PKT_ACKED,
    PKT_TIMEOUT
};

enum {
    CFLIE_INIT_SEARCH = 0,
    CFLIE_INIT_DATA,
    CFLIE_SEARCH,
    CFLIE_DATA = 0x10
};

public:
    RFProtocolCFlie():RFProtocol(RFProtocol::TX_NRF24L01, RFProtocol::PROTO_NRF24L01_CFLIE) { }
    RFProtocolCFlie(u32 id):RFProtocol(id) { }
    ~RFProtocolCFlie() { close(); }

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
    u8   checkStatus(void);
    void initRxTxAddr(void);
    void init1(void);
    u16  getChannel(CH_T id);
    void setRateAndCh(u8 rate, u8 channel);
    void sendSearchPacket(void);
    void frac2float(s32 n, float* res);
    void sendCmdPacket(void);

// variables
    DeviceNRF24L01  mDev;
    u8   mDataRate;
    u8   mCurChan;
    u8   mPacketBuf[MAX_PACKET_SIZE];

    u16  mBindCtr;
    u32  mPacketCtr;
    u8   mRxTxAddrBuf[ADDR_BUF_SIZE];
    u8   mState;
    
protected:

};

#endif
