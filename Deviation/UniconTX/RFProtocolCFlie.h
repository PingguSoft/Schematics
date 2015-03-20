#ifndef _PROTOCOL_CFLIE_
#define _PROTOCOL_CFLIE_

#include "DeviceNRF24L01.h"
#include "RFProtocol.h"

class RFProtocolCFlie : public RFProtocol
{
#define PAYLOADSIZE          8
#define MAX_PACKET_SIZE     15
#define MAX_BIND_COUNT      60

#define PACKET_PERIOD_uS 10000
#define PACKET_CHK_uS     1000
#define INITIAL_WAIT_uS  50000

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
    RFProtocolCFlie(u32 id):RFProtocol(id) { }
    ~RFProtocolCFlie() { close(); }

// for protocol
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
    void setRateAndCh(u8 rate, u8 channel);
    void sendSearchPacket(void);
    void frac2float(s32 n, float* res);
    void sendCmdPacket(void);

// variables
    DeviceNRF24L01  mDev;

    u32  mPacketCtr;
    u16  mBindCtr;
    u8   mPacketBuf[MAX_PACKET_SIZE];    
    u8   mRxTxAddrBuf[ADDR_BUF_SIZE];
    
    u8   mDataRate;
    u8   mCurChan;
    u8   mState;

protected:

};

#endif
