// For Arduino 1.0 and earlier
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "Protocol.h"

Protocol::Protocol(TX_MOD_T module, PROTO_T protocol) 
{
    mModule   = module;
    mProtocol = protocol;
}

void Protocol::injectDir(u8 throttle, u8 rudder, u8 elevator, u8 aileron)
{ 
    mByteThrottle = throttle;
    mByteRudder   = rudder;
    mByteElevator = elevator;
    mByteAileron  = aileron;
}

void Protocol::injectTrim(u8 rudder, u8 elevator, u8 aileron) 
{
    mByteRudderTrim = rudder;
    mByteElevatorTrim = elevator;
    mByteAileronTrim = aileron;
}
