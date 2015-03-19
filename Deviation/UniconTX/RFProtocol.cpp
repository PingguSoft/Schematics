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

    mTmrState = -1;
    mTXPower  = TXPOWER_10mW;
}

RFProtocol::RFProtocol(u32 id)
{
    mProtoID = id;
    initVars();
}

RFProtocol::~RFProtocol()
{
}

void RFProtocol::loop(void)
{
    update();
}

int RFProtocol::init(void)
{
    return 0;
}

int RFProtocol::close(void)
{
    if (mTmrState > 0)
        stop(mTmrState);
    return 0;
}

int RFProtocol::reset(void)
{
    return 0;
}

int RFProtocol::getChannels(void)
{
    return 0;
}

int RFProtocol::setRFPower(u8 power)
{
    mTXPower = (power | 0x80);
    return 0;
}

u8 RFProtocol::getRFPower(void)
{
    return (mTXPower & 0x7f);
}

bool RFProtocol::isRFPowerUpdated(void)
{
    return (mTXPower & 0x80);
}

void RFProtocol::clearRFPowerUpdated(void)
{
    mTXPower &= 0x7f;
}

int RFProtocol::getInfo(s8 id, u8 *data)
{
    return 0;
}

void RFProtocol::test(s8 id)
{
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

s16 RFProtocol::getControl(u8 ch)
{
    return mBufControls[ch];
}

void RFProtocol::handleTimer(s8 id)
{
    u16 nextTime;

    if (id == mTmrState) {
        nextTime = callState();
        if (nextTime > 0)
            mTmrState = after(nextTime);
        else
            stop(mTmrState);
    }
}

void RFProtocol::startState(unsigned long period)
{
    mTmrState = after(period);
}
