// For Arduino 1.0 and earlier
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "Protocol.h"

s32 Protocol::mIntChannels[MAX_CHANNEL];
u8 Protocol::mByteTrims[MAX_TRIM];

Protocol::Protocol(TX_MOD_T module, PROTO_T protocol)
{
    mModule   = module;
    mProtocol = protocol;

    memset(Protocol::mIntChannels, 0, sizeof(Protocol::mIntChannels));
//    for (u8 i = 0; i < MAX_CHANNEL; i++)
//      Protocol::mIntChannels[i] = -5000;
    Protocol::mIntChannels[CH_THROTTLE] = CHAN_MIN_VALUE;

    Protocol::mByteTrims[TRIM_RUDDER] = 0x0 >> 1;
    Protocol::mByteTrims[TRIM_ELEVATOR] = 0x0 >> 1;
    Protocol::mByteTrims[TRIM_AILERON] = 0x0 >> 1;
}

void Protocol::injectControl(CH_T ch, s32 val)
{
    mIntChannels[ch] = val;
}

void Protocol::injectControls(s32 *data, int size)
{
    for (int i = 0; i < size; i++)
        mIntChannels[i] = *data++;
}

void Protocol::injectTrim(TRIM_T trim, u8 val)
{
    mByteTrims[trim] =  val;
}

void Protocol::injectTrims(u8 *data)
{
    for (int i = 0; i < MAX_TRIM; i++)
        mByteTrims[i] = *data;
}

u32 Protocol::getControl(CH_T ch)
{
    return mIntChannels[ch];
}

u8 Protocol::getTrim(TRIM_T trim)
{
    return mByteTrims[trim];
}

