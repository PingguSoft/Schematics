#ifndef Protocol_h
#define Protocol_h

#include "Common.h"
#include "Timer.h"

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

class Protocol
{
public:
    Protocol(TX_MOD_T module, PROTO_T protocol);
    TX_MOD_T getModule()      { return mModule; };
    PROTO_T  getProtocol()    { return mProtocol; }
    
    static void injectDir(u8 throttle, u8 rudder, u8 elevator, u8 aileron);
    static void injectTrim(u8 rudder, u8 elevator, u8 aileron);
    static void update()    { mTimer.update(); };
    static Timer getTimer() { return mTimer;   };
    
private:
    static Timer mTimer;
    static u8  tx_power;
    static u8  mByteThrottle, mByteRudder, mByteElevator, mByteAileron;
    static u8  mByteRudderTrim, mByteElevatorTrim, mByteAileronTrim;

protected:

    virtual int init(u32 id) = 0;
    virtual int deinit() = 0;
    virtual int reset() = 0;
    virtual int bind(u32 id) = 0;
    virtual int getChannels() = 0;
    virtual int setPower(int power) = 0;
    
    TX_MOD_T mModule;
    PROTO_T  mProtocol;
};

#endif
