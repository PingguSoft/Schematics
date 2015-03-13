#include <Arduino.h>
#include <avr/pgmspace.h>
#include <SPI.h>

#include "common.h"
#include "utils.h"
#include "RFProtocolSyma.h"
#include "RFProtocolYD717.h"
#include "SerialProtocol.h"

SerialProtocol  mSerial;
RFProtocol      *mRFProto = NULL;

u32 serialCallback(u8 cmd, u8 *data, u8 size)
{
    u32 id;
    u8  ret = 0;

    switch (cmd) {
        case SerialProtocol::CMD_SET_RFPROTOCOL:
            if (mRFProto) {
                delete mRFProto;
                mRFProto = NULL;
            }
            id = *(u32*)data;
            if (RFProtocol::getModule(id) == RFProtocol::TX_NRF24L01) {
                switch (RFProtocol::getProtocol(id)) {
                    case RFProtocol::PROTO_NRF24L01_SYMAX:
                        mRFProto = new RFProtocolSyma(id);
                        ret = 1;
                        break;

                    case RFProtocol::PROTO_NRF24L01_YD717:
                        mRFProto = new RFProtocolYD717(id);
                        ret = 1;
                        break;
                }
            }
            mSerial.sendResponse(true, cmd, (u8*)&ret, sizeof(ret));
            break;

        case SerialProtocol::CMD_START_RF:
            id = *(u32*)data;
            if (mRFProto) {
                mRFProto->setControllerID(id);
                mRFProto->init();
                ret = 1;
            }
            mSerial.sendResponse(true, cmd, (u8*)&ret, sizeof(ret));
            break;

        case SerialProtocol::CMD_STOP_RF:
            if (mRFProto) {
                mRFProto->close();
                delete mRFProto;
                mRFProto = NULL;
                ret = 1;
            }
            mSerial.sendResponse(true, cmd, (u8*)&ret, sizeof(ret));
            break;

        case SerialProtocol::CMD_INJECT_CONTROLS:
            if (mRFProto) {
                mRFProto->injectControls((s16*)data, size >> 1);
                ret = 1;
            }
            mSerial.sendResponse(true, cmd, (u8*)&ret, sizeof(ret));
            break;

        case SerialProtocol::CMD_GET_INFO:
            u8 buf[5];
            u8 size = 0;

            buf[0] = *data;
            if (mRFProto) {
                mRFProto->getInfo(*data, &buf[1], &size);
            }
            mSerial.sendResponse(true, cmd, buf, size + 1);
            break;
    }
}

void setup()
{
	mSerial.begin(57600);
    mSerial.setCallback(serialCallback);
}

int freeRam() {
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

void loop()
{
    mSerial.handleRX();
    if (mRFProto)
        mRFProto->loop();
}

