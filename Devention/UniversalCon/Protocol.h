#ifndef Protocol_h
#define Protocol_h

#include "Common.h"


typedef enum {
    TX_MOD_CYRF6936,
    TX_MOD_A7105,
    TX_MOD_NRF24L01,
    TX_MOD_LAST,
} TX_MOD_T;

typedef enum {
    PROTO_V202,
    PROTO_SLT,
    PROTO_HiSky,
    PROTO_YD717,
    PROTO_SymaX,
    PROTO_CFlie,
    PROTO_H377,
    PROTO_HM830,
    PROTO_KN,
    PROTO_ESKY150,
    PROTO_ESKY
} PROTO_T;

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

#define CHAN_MAX_VALUE 10000
#define CHAN_MIN_VALUE -10000

class Protocol
{
public:
    Protocol(TX_MOD_T module, PROTO_T protocol);
    TX_MOD_T getModule()      { return mModule; };
    PROTO_T  getProtocol()    { return mProtocol; }

    static void  injectControl(CH_T ch, s32 val);
    static void  injectControls(s32 *data, int size);
    static void  injectTrim(TRIM_T trim, u8 val);
    static void  injectTrims(u8 *data);
    static u32   getControl(CH_T ch);
    static u8    getTrim(TRIM_T trim);

    virtual void update() = 0;
    virtual int  init(u32 id) = 0;
    virtual int  deinit() = 0;
    virtual int  reset() = 0;
    virtual int  bind(u32 id) = 0;
    virtual int  getChannels() = 0;
    virtual int  setPower(int power) = 0;

private:
    static u8  tx_power;
    static s32 mIntChannels[MAX_CHANNEL];
    static u8  mByteTrims[MAX_TRIM];

protected:


    TX_MOD_T mModule;
    PROTO_T  mProtocol;
};

#endif
