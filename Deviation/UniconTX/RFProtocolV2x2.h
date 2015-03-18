#ifndef _PROTOCOL_V2x2_
#define _PROTOCOL_V2x2_

#include "DeviceNRF24L01.h"
#include "RFProtocol.h"

class RFProtocolV2x2 : public RFProtocol
{

#define MAX_PACKET_SIZE     16
#define MAX_BIND_COUNT    1000

#define PACKET_PERIOD_MS     4
#define PACKET_CHKTIME       1
#define INITIAL_WAIT_MS     50
#define FIRST_PACKET_MS     12
#define ADDR_BUF_SIZE        3

// Every second
#define BLINK_COUNT         250
// ~ every 0.25 sec
#define BLINK_COUNT_MIN     64
// ~ every 2 sec
#define BLINK_COUNT_MAX     512

enum {
    V2x2_FLAG_CAMERA = 0x01, // also automatic Missile Launcher and Hoist in one direction
    V2x2_FLAG_VIDEO  = 0x02, // also Sprayer, Bubbler, Missile Launcher(1), and Hoist in the other dir.
    V2x2_FLAG_FLIP   = 0x04,
    V2x2_FLAG_UNK9   = 0x08,
    V2x2_FLAG_LED    = 0x10,
    V2x2_FLAG_UNK10  = 0x20,
    V2x2_FLAG_BIND   = 0xC0
};

enum {
    V202_INIT2 = 0,
    V202_INIT2_NO_BIND,
    V202_BIND1,
    V202_BIND2,
    V202_DATA  = 0x10
};

enum {
    STARTBIND_NO  = 0,
    STARTBIND_YES = 1,
};

enum {
    USEBLINK_NO  = 0,
    USEBLINK_YES = 1,
};

// Packet ack status values
enum {
    PKT_PENDING = 0,
    PKT_ACKED,
    PKT_TIMEOUT
};

#define MAX_RF_CHANNELS     17

public:
    RFProtocolV2x2(u32 id):RFProtocol(id) { }
    ~RFProtocolV2x2() { close(); }

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
    u8   getCheckSum(u8 *data);
    u8   checkStatus(void);
    u8   getChannel(CH_T id);
    void getControls(u8* throttle, u8* rudder, u8* elevator, u8* aileron, u8* flags, u16 *led_blink);
    void sendPacket(u8 bind);
    void initRxTxAddr(void);
    void init1(void);
    void init2(void);
    void setTxID(u32 id);

// variables
    DeviceNRF24L01  mDev;
    u8   mCurChan;
    u8   mChannelBuf[MAX_RF_CHANNELS];
    u8   mPacketBuf[MAX_PACKET_SIZE];
    u16  mBindCtr;
    u32  mPacketCtr;
    u8   mAuxFlag;
    u16  mLedBlinkCtr;
    u8   mRxTxAddrBuf[ADDR_BUF_SIZE];
    u8   mState;
    u8   mPacketSent;
protected:

};

#endif
