// For Arduino 1.0 and earlier
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "RFProtocol.h"
#include "utils.h"


void RFProtocol::initVars(void)
{
    memset(mBufControls, 0, sizeof(mBufControls));
    mBufControls[CH_THROTTLE] = CHAN_MIN_VALUE;

    mBufTrims[TRIM_RUDDER] = 0x0 >> 1;
    mBufTrims[TRIM_ELEVATOR] = 0x0 >> 1;
    mBufTrims[TRIM_AILERON] = 0x0 >> 1;
}

RFProtocol::RFProtocol(u32 id)
{
    mProtoID = id;
    initVars();
}

RFProtocol::RFProtocol(u8 module, u8 proto)
{ 
    mProtoID = ((u32)module << 16 | (u32)proto << 8);
    initVars();
}

void RFProtocol::injectControl(u8 ch, s16 val)
{
    mBufControls[ch] = val;
}

void RFProtocol::injectControls(s16 *data, int size)
{
    for (int i = 0; i < size; i++)
        mBufControls[i] = *data++;
}

void RFProtocol::injectTrim(u8 trim, u8 val)
{
    mBufTrims[trim] =  val;
}

void RFProtocol::injectTrims(u8 *data)
{
    for (int i = 0; i < MAX_TRIM; i++)
        mBufTrims[i] = *data;
}

s16 RFProtocol::getControl(u8 ch)
{
    return mBufControls[ch];
}

u8 RFProtocol::getTrim(u8 trim)
{
    return mBufTrims[trim];
}

