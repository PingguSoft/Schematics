#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_

#include "Common.h"
#include "utils.h"
#include "Timer.h"

class RFProtocol : public Timer
{
public:
    #define CHAN_MAX_VALUE 500
    #define CHAN_MIN_VALUE -500

    typedef enum {
        TX_NRF24L01,
        TX_A7105,
        TX_CYRF6936,
    } TX_T;

    typedef enum {
        PROTO_NRF24L01_V2x2,
        PROTO_NRF24L01_HISKY,
        PROTO_NRF24L01_YD717,
        PROTO_NRF24L01_SYMAX,
        PROTO_NRF24L01_CFLIE,
    } PROTO_NRF24L01_T;

    typedef enum {
        PROTO_A7105_FLYSKY,
        PROTO_A7105_HUBSAN,
    } PROTO_A7105_T;

    typedef enum {
        PROTO_CYRF6936_DEVO,
    } PROTO_CYRF6936_T;

    typedef enum {
        CH_THROTTLE = 0,
        CH_RUDDER,
        CH_ELEVATOR,
        CH_AILERON,
        CH_AUX1,
        CH_AUX2,
        CH_AUX3,
        CH_AUX4,
        CH_AUX5,
        CH_AUX6,
        CH_AUX7,
        CH_AUX8,
        MAX_CHANNEL = CH_AUX8
    } CH_T;

    typedef enum {
        TRIM_RUDDER,
        TRIM_ELEVATOR,
        TRIM_AILERON,
        MAX_TRIM = TRIM_AILERON
    } TRIM_T;

    typedef enum {
        INFO_STATE,
        INFO_CHANNEL,
        INFO_PACKET_CTR
    } INFO_T;

    // utility functions
    static u32   buildID(u8 module, u8 proto, u8 option)  { return ((u32)module << 16 | (u32)proto << 8 | option); }
    static u8    getModule(u32 id)      { return (id >> 16) & 0xff; }
    static u8    getProtocol(u32 id)    { return (id >> 8) & 0xff;  }
    static u8    getProtocolOpt(u32 id) { return id & 0xff;         }


    RFProtocol(u32 id);
    RFProtocol(u8 module, u8 proto);
    virtual ~RFProtocol()           { }

    u32  getProtoID(void)           { return mProtoID; }
    u8   getModule(void)            { return (mProtoID >> 16) & 0xff; }
    u8   getProtocol(void)          { return (mProtoID >> 8) & 0xff;  }
    u8   getProtocolOpt(void)       { return mProtoID & 0xff; }
    void setControllerID(u32 id)    { mConID = id;     }
    u32  getControllerID()          { return mConID;   }

    void injectControl(u8 ch, s16 val);
    void injectControls(s16 *data, int size);
    s16  getControl(u8 ch);

    void injectTrim(u8 trim, u8 val);
    void injectTrims(u8 *data);
    u8   getTrim(u8 trim);

// for timer
    virtual void handleTimer(s8 id) = 0;

// for protocol
    virtual void loop(void)    { }
    virtual int  init(void)    { return 0; }
    virtual int  close(void)   { return 0; }
    virtual int  reset(void)   { return 0; }
    virtual int  getChannels(void)   { return 0; }
    virtual int  setPower(int power) { return 0; }
    virtual int  getInfo(s8 id, u8 *data) { return 0; }
    virtual void test(s8 id) { }

private:
    void initVars();

    s16  mBufControls[MAX_CHANNEL];
    u8   mBufTrims[MAX_TRIM];
    u32  mProtoID;
    u32  mConID;
};

#endif
