#include <Arduino.h>
#include <avr/pgmspace.h>
#include <SPI.h>

#include "common.h"
#include "utils.h"
#include "RFProtocolSyma.h"
#include "RFProtocolYD717.h"
#include "RFProtocolV2x2.h"
#include "RFProtocolHiSky.h"
#include "RFProtocolCFlie.h"
#include "RFProtocolDevo.h"
#include "SerialProtocol.h"


static SerialProtocol  mSerial;
static RFProtocol      *mRFProto = NULL;

u32 serialCallback(u8 cmd, u8 *data, u8 size)
{
    u32 id;
    u8  ret = 0;
    u8  buf[5];
    u8  sz = 0;

    switch (cmd) {
        case SerialProtocol::CMD_SET_RFPROTOCOL:
            if (mRFProto) {
                delete mRFProto;
                mRFProto = NULL;
            }
            
            id = *(u32*)data;
            switch (RFProtocol::getModule(id)) {
                case RFProtocol::TX_NRF24L01: {
                    switch (RFProtocol::getProtocol(id)) {
                        case RFProtocol::PROTO_NRF24L01_SYMAX:
                            mRFProto = new RFProtocolSyma(id);
                            ret = 1;
                            break;

                        case RFProtocol::PROTO_NRF24L01_YD717:
                            mRFProto = new RFProtocolYD717(id);
                            ret = 1;
                            break;

                        case RFProtocol::PROTO_NRF24L01_V2x2:
                            mRFProto = new RFProtocolV2x2(id);
                            ret = 1;
                            break;

                        case RFProtocol::PROTO_NRF24L01_HISKY:
                            mRFProto = new RFProtocolHiSky(id);
                            ret = 1;
                            break;

                        case RFProtocol::PROTO_NRF24L01_CFLIE:
                            mRFProto = new RFProtocolCFlie(id);
                            ret = 1;
                            break;
                    }
                }
                break;

                case RFProtocol::TX_CYRF6936:
                    mRFProto = new RFProtocolDevo(id);
                    ret = 1;
                break;
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
            buf[0] = *data;
            if (mRFProto) {
                sz = mRFProto->getInfo(buf[0], &buf[1]);
            }
            mSerial.sendResponse(true, cmd, buf, sz + 1);
            break;

        case SerialProtocol::CMD_GET_FREE_RAM:
            u16 ram = freeRam();
            mSerial.sendResponse(true, cmd, (u8*)&ram, sizeof(ram));
            break;
    }
    return ret;
}

void setup()
{
    mSerial.begin(57600);
    mSerial.setCallback(serialCallback);
}

void loop()
{
    mSerial.handleRX();
    if (mRFProto)
        mRFProto->loop();
}

int freeRam() {
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
