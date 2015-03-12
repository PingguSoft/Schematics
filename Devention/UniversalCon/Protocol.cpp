// For Arduino 1.0 and earlier
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "Protocol.h"
#include "utils.h"

s32 Protocol::mIntContolsBuf[MAX_CHANNEL];
u8  Protocol::mByteTrimsBuf[MAX_TRIM];

Protocol::Protocol(TX_MOD_T module, PROTO_T protocol)
{
    mModule   = module;
    mProtocol = protocol;

    memset(Protocol::mIntContolsBuf, 0, sizeof(Protocol::mIntContolsBuf));
    Protocol::mIntContolsBuf[CH_THROTTLE] = CHAN_MIN_VALUE;

    Protocol::mByteTrimsBuf[TRIM_RUDDER] = 0x0 >> 1;
    Protocol::mByteTrimsBuf[TRIM_ELEVATOR] = 0x0 >> 1;
    Protocol::mByteTrimsBuf[TRIM_AILERON] = 0x0 >> 1;
}

void Protocol::injectControl(CH_T ch, s32 val)
{
    mIntContolsBuf[ch] = val;
}

void Protocol::injectControls(s32 *data, int size)
{
    for (int i = 0; i < size; i++)
        mIntContolsBuf[i] = *data++;
}

void Protocol::injectTrim(TRIM_T trim, u8 val)
{
    mByteTrimsBuf[trim] =  val;
}

void Protocol::injectTrims(u8 *data)
{
    for (int i = 0; i < MAX_TRIM; i++)
        mByteTrimsBuf[i] = *data;
}

u32 Protocol::getControl(CH_T ch)
{
    return mIntContolsBuf[ch];
}

u8 Protocol::getTrim(TRIM_T trim)
{
    return mByteTrimsBuf[trim];
}

