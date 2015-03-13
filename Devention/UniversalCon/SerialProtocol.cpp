// For Arduino 1.0 and earlier
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "SerialProtocol.h"
#include "utils.h"


SerialProtocol::SerialProtocol()
{
}

SerialProtocol::~SerialProtocol()
{
}

void SerialProtocol::setCallback(u32 (*callback)(u8 cmd, u8 *data, u8 size))
{
    mCallback = callback;
}

void SerialProtocol::sendResponse(bool ok, u8 cmd, u8 *data, u8 size)
{
    u8  txBuf[6 + size];
    u8  pos = 0;

    txBuf[0] = '$';
    txBuf[1] = 'M';
    txBuf[2] = (ok ? '>' : '!');
    txBuf[3] = size;
    txBuf[4] = cmd;
    for (u8 i = 5; i < 5 + size; i++) {
        txBuf[i] = *data++;
    }

    u8 checksum = 0;
    for (u8 i = 3; i < 5 + size; i++) {
        checksum ^= txBuf[i];
    }
    txBuf[5 + size] = checksum;
    Serial.write(txBuf, 6 + size);
}


void SerialProtocol::evalCommand(u8 cmd, u8 *data, u8 size)
{
    static u8 batt = 0;

    switch (cmd) {
        case CMD_TEST:
            u8 data[7];
            data[0] = batt++;
            sendResponse(true, cmd, data, 7); 
            break;

        default:
            if (mCallback)
                (*mCallback)(cmd, data, size);
            break;
    }
}

void SerialProtocol::handleRX(void)
{
    u8 rxSize = Serial.available();
    
    if (rxSize < 1)
        return;

    while (rxSize--) {
        u8 ch = Serial.read();
        
        switch (mState) {
            case STATE_IDLE:
                if (ch == '$')
                    mState = STATE_HEADER_START;
                break;

            case STATE_HEADER_START:
                mState = (ch == 'M') ? STATE_HEADER_M : STATE_IDLE;
                break;

            case STATE_HEADER_M:
                mState = (ch == '<') ? STATE_HEADER_ARROW : STATE_IDLE;
                break;

            case STATE_HEADER_ARROW:
                if (ch > MAX_RX_BUF) {      // now we are expecting the payload size
                    mState = STATE_IDLE;
                    continue;
                }
                mDataSize = ch;
                mCheckSum = ch;
                mOffset   = 0;
                mState    = STATE_HEADER_SIZE;
                break;

            case STATE_HEADER_SIZE:
                mCmd       = ch;
                mCheckSum ^= ch;
                mState     = STATE_HEADER_CMD;
                break;

            case STATE_HEADER_CMD:
                if (mOffset < mDataSize) {
                    mCheckSum        ^= ch;
                    mRxBuf[mOffset++] = ch;
                } else {
                    if (mCheckSum == ch)
                        evalCommand(mCmd, mRxBuf, mDataSize);
                    mState = STATE_IDLE;
                }
                break;
        }
    }
}
