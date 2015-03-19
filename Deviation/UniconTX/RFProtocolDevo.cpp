#include <SPI.h>
#include "RFProtocolDevo.h"
#include "utils.h"

static const PROGMEM u8 SOPCODES[][8] = {
    /* Note these are in order transmitted (LSB 1st) */
    /* 0 */ {0x3C,0x37,0xCC,0x91,0xE2,0xF8,0xCC,0x91}, //0x91CCF8E291CC373C
    /* 1 */ {0x9B,0xC5,0xA1,0x0F,0xAD,0x39,0xA2,0x0F}, //0x0FA239AD0FA1C59B
    /* 2 */ {0xEF,0x64,0xB0,0x2A,0xD2,0x8F,0xB1,0x2A}, //0x2AB18FD22AB064EF
    /* 3 */ {0x66,0xCD,0x7C,0x50,0xDD,0x26,0x7C,0x50}, //0x507C26DD507CCD66
    /* 4 */ {0x5C,0xE1,0xF6,0x44,0xAD,0x16,0xF6,0x44}, //0x44F616AD44F6E15C
    /* 5 */ {0x5A,0xCC,0xAE,0x46,0xB6,0x31,0xAE,0x46}, //0x46AE31B646AECC5A
    /* 6 */ {0xA1,0x78,0xDC,0x3C,0x9E,0x82,0xDC,0x3C}, //0x3CDC829E3CDC78A1
    /* 7 */ {0xB9,0x8E,0x19,0x74,0x6F,0x65,0x18,0x74}, //0x7418656F74198EB9
    /* 8 */ {0xDF,0xB1,0xC0,0x49,0x62,0xDF,0xC1,0x49}, //0x49C1DF6249C0B1DF
    /* 9 */ {0x97,0xE5,0x14,0x72,0x7F,0x1A,0x14,0x72}, //0x72141A7F7214E597
};

void RFProtocolDevo::buildScramblePacket(void)
{
    u8 i;
    for(i = 0; i < 15; i++) {
        mPacketBuf[i + 1] ^= mMFGID[i % 4];
    }
}

void RFProtocolDevo::addPacketSuffix(void)
{
    u8 bind_state;
    
    if (use_fixed_id) {
        if (mBindCtr > 0) {
            bind_state = 0xc0;
        } else {
            bind_state = 0x80;
        }
    } else {
        bind_state = 0x00;
    }
    mPacketBuf[10] = bind_state | (PKTS_PER_CHANNEL - mPacketCtr - 1);
    mPacketBuf[11] = *(radio_ch_ptr + 1);
    mPacketBuf[12] = *(radio_ch_ptr + 2);
    mPacketBuf[13] = fixed_id  & 0xff;
    mPacketBuf[14] = (fixed_id >> 8) & 0xff;
    mPacketBuf[15] = (fixed_id >> 16) & 0xff;
}

void RFProtocolDevo::buildBeaconPacket(int upper)
{
    mPacketBuf[0] = ((mConChanCnt << 4) | 0x07);
    u8 enable = 0;
    int max = 8;
    int offset = 0;
    if (upper) {
        mPacketBuf[0] += 1;
        max = 4;
        offset = 8;
    }
    for(int i = 0; i < max; i++) {
#ifdef BOGUS
        if (i + offset < Model.mConChanCnt && Model.limits[i+offset].flags & CH_FAILSAFE_EN) {
            enable |= 0x80 >> i;
            mPacketBuf[i+1] = Model.limits[i+offset].failsafe;
        } else {
#endif /* BOGUS */
            mPacketBuf[i+1] = 0;
#ifdef BOGUS
        }
#endif /* BOGUS */
    }
    mPacketBuf[9] = enable;
    addPacketSuffix();
}

void RFProtocolDevo::buildBindPacket(void)
{
    mPacketBuf[0] = (mConChanCnt << 4) | 0x0a;
    mPacketBuf[1] = mBindCtr & 0xff;
    mPacketBuf[2] = (mBindCtr >> 8);
    mPacketBuf[3] = *radio_ch_ptr;
    mPacketBuf[4] = *(radio_ch_ptr + 1);
    mPacketBuf[5] = *(radio_ch_ptr + 2);
    mPacketBuf[6] = mMFGID[0];
    mPacketBuf[7] = mMFGID[1];
    mPacketBuf[8] = mMFGID[2];
    mPacketBuf[9] = mMFGID[3];
    addPacketSuffix();
    //The fixed-id portion is scrambled in the bind mPacketBuf
    //I assume it is ignored
    mPacketBuf[13] ^= mMFGID[0];
    mPacketBuf[14] ^= mMFGID[1];
    mPacketBuf[15] ^= mMFGID[2];
}

void RFProtocolDevo::buildDataPacket(void)
{
    s32 value;
    u8 i;
    u8 tblCH[] = { CH_ELEVATOR, CH_AILERON, CH_THROTTLE, CH_RUDDER };
    u8 sign = 0x0b;
    u8 idx;
    
    mPacketBuf[0] = (mConChanCnt << 4) | (0x0b + mConChanIdx);
    for (i = 0; i < 4; i++) {
        if (mConChanIdx == 0) {
            idx = tblCH[i];
        } else {
            idx = i;
        }
        value = (s32)getControl(idx) * 0x640 / CHAN_MAX_VALUE;
        
        if(value < 0) {
            value = -value;
            sign |= 1 << (7 - i);
        }
        mPacketBuf[2 * i + 1] = value & 0xff;
        mPacketBuf[2 * i + 2] = (value >> 8) & 0xff;
    }
    mPacketBuf[9] = sign;

    mConChanIdx = mConChanIdx + 1;
    if (mConChanIdx * 4 >= mConChanCnt)
        mConChanIdx = 0;
    addPacketSuffix();
}

s32 RFProtocolDevo::float_to_int(u8 *ptr)
{
    s32 value = 0;
    int seen_decimal = 0;
    for(int i = 0; i < 7; i++) {
        if(ptr[i] == '.') {
            value *= 1000;
            seen_decimal = 100;
            continue;
        }
        if(ptr[i] == 0)
            break;
        if(seen_decimal) {
            value += (ptr[i] - '0') * seen_decimal;
            seen_decimal /= 10;
            if(! seen_decimal)
                break;
        } else {
            value = value * 10 + (ptr[i] - '0');
        }
    }
    return value;
}

void RFProtocolDevo::parseTelemetryPacket(u8 *mPacketBuf)
{
#ifdef BOGUS

    static const u8 voltpkt[] = {
            TELEM_DEVO_VOLT1, TELEM_DEVO_VOLT2, TELEM_DEVO_VOLT3,
            TELEM_DEVO_RPM1, TELEM_DEVO_RPM2, 0
        };
    static const u8 temppkt[] = {
            TELEM_DEVO_TEMP1, TELEM_DEVO_TEMP2, TELEM_DEVO_TEMP3, TELEM_DEVO_TEMP4, 0
        };
    static const u8 gpslongpkt[] = { TELEM_GPS_LONG, 0};
    static const u8 gpslatpkt[] = { TELEM_GPS_LAT, 0};
    static const u8 gpsaltpkt[] = { TELEM_GPS_ALT, 0};
    static const u8 gpsspeedpkt[] = { TELEM_GPS_SPEED, 0};
    static const u8 gpstimepkt[] = { TELEM_GPS_TIME, 0};

    if((mPacketBuf[0] & 0xF0) != 0x30)
        return;
    const u8 *update = NULL;
    buildScramblePacket(); //This will unscramble the mPacketBuf
    if (mPacketBuf[13] != (fixed_id  & 0xff)
        || mPacketBuf[14] != ((fixed_id >> 8) & 0xff)
        || mPacketBuf[15] != ((fixed_id >> 16) & 0xff))
    {
        return;
    }
    //if (mPacketBuf[0] < 0x37) {
    //    memcpy(Telemetry.line[mPacketBuf[0]-0x30], mPacketBuf+1, 12);
    //}
    if (mPacketBuf[0] == TELEMETRY_ENABLE) {
        update = voltpkt;
        Telemetry.p.devo.volt[0] = mPacketBuf[1]; //In 1/10 of Volts
        Telemetry.p.devo.volt[1] = mPacketBuf[3]; //In 1/10 of Volts
        Telemetry.p.devo.volt[2] = mPacketBuf[5]; //In 1/10 of Volts
        Telemetry.p.devo.rpm[0]  = mPacketBuf[7] * 120; //In RPM
        Telemetry.p.devo.rpm[1]  = mPacketBuf[9] * 120; //In RPM
    }
    if (mPacketBuf[0] == 0x31) {
        update = temppkt;
        Telemetry.p.devo.temp[0] = mPacketBuf[1] == 0xff ? 0 : mPacketBuf[1] - 20; //In degrees-C
        Telemetry.p.devo.temp[1] = mPacketBuf[2] == 0xff ? 0 : mPacketBuf[2] - 20; //In degrees-C
        Telemetry.p.devo.temp[2] = mPacketBuf[3] == 0xff ? 0 : mPacketBuf[3] - 20; //In degrees-C
        Telemetry.p.devo.temp[3] = mPacketBuf[4] == 0xff ? 0 : mPacketBuf[4] - 20; //In degrees-C
    }
    /* GPS Data
       32: 30333032302e3832373045fb  = 030°20.8270E
       33: 353935342e373737364e0700  = 59°54.776N
       34: 31322e380000004d4d4e45fb  = 12.8 MMNE (altitude maybe)?
       35: 000000000000302e30300000  = 0.00 (probably speed)
       36: 313832353532313531303132  = 2012-10-15 18:25:52 (UTC)
    */
    if (mPacketBuf[0] == 0x32) {
        update = gpslongpkt;
        Telemetry.gps.longitude = ((mPacketBuf[1]-'0') * 100 + (mPacketBuf[2]-'0') * 10 + (mPacketBuf[3]-'0')) * 3600000
                                  + ((mPacketBuf[4]-'0') * 10 + (mPacketBuf[5]-'0')) * 60000
                                  + ((mPacketBuf[7]-'0') * 1000 + (mPacketBuf[8]-'0') * 100
                                     + (mPacketBuf[9]-'0') * 10 + (mPacketBuf[10]-'0')) * 6;
        if (mPacketBuf[11] == 'W')
            Telemetry.gps.longitude *= -1;
    }
    if (mPacketBuf[0] == 0x33) {
        update = gpslatpkt;
        Telemetry.gps.latitude = ((mPacketBuf[1]-'0') * 10 + (mPacketBuf[2]-'0')) * 3600000
                                  + ((mPacketBuf[3]-'0') * 10 + (mPacketBuf[4]-'0')) * 60000
                                  + ((mPacketBuf[6]-'0') * 1000 + (mPacketBuf[7]-'0') * 100
                                     + (mPacketBuf[8]-'0') * 10 + (mPacketBuf[9]-'0')) * 6;
        if (mPacketBuf[10] == 'S')
            Telemetry.gps.latitude *= -1;
    }
    if (mPacketBuf[0] == 0x34) {
        update = gpsaltpkt;
        Telemetry.gps.altitude = float_to_int(mPacketBuf+1);
    }
    if (mPacketBuf[0] == 0x35) {
        update = gpsspeedpkt;
        Telemetry.gps.velocity = float_to_int(mPacketBuf+7);
    }
    if (mPacketBuf[0] == 0x36) {
        update = gpstimepkt;
        u8 hour  = (mPacketBuf[1]-'0') * 10 + (mPacketBuf[2]-'0');
        u8 min   = (mPacketBuf[3]-'0') * 10 + (mPacketBuf[4]-'0');
        u8 sec   = (mPacketBuf[5]-'0') * 10 + (mPacketBuf[6]-'0');
        u8 day   = (mPacketBuf[7]-'0') * 10 + (mPacketBuf[8]-'0');
        u8 month = (mPacketBuf[9]-'0') * 10 + (mPacketBuf[10]-'0');
        u8 year  = (mPacketBuf[11]-'0') * 10 + (mPacketBuf[12]-'0'); // + 2000
        Telemetry.gps.time = ((year & 0x3F) << 26)
                           | ((month & 0x0F) << 22)
                           | ((day & 0x1F) << 17)
                           | ((hour & 0x1F) << 12)
                           | ((min & 0x3F) << 6)
                           | ((sec & 0x3F) << 0);
    }

#endif /* BOGUS */
}

void RFProtocolDevo::setBoundSOPCodes(void)
{
    /* crc == 0 isn't allowed, so use 1 if the math results in 0 */
    u8 crc = (mMFGID[0] + (mMFGID[1] >> 6) + mMFGID[2]);
    if(! crc)
        crc = 1;
    u8 sopidx = (0xff &((mMFGID[0] << 2) + mMFGID[1] + mMFGID[2])) % 10;
    mDev.setTxRxMode(TX_EN);
    mDev.setCRCSeed((crc << 8) + crc);
    mDev.setSOPCode(SOPCODES[sopidx]);
    mDev.writeReg(CYRF_03_TX_CFG, 0x08 | getRFPower());
}

void RFProtocolDevo::init1(void)
{
    /* Initialise CYRF chip */
    mDev.reset();

    mDev.readMfgID(mMFGID);
    mDev.setTxRxMode(TX_EN);
    mDev.setCRCSeed(0x0000);
    mDev.setSOPCode(SOPCODES[0]);
    setRadioChannels();
    
    mDev.writeReg(CYRF_1D_MODE_OVERRIDE, 0x38);
    mDev.writeReg(CYRF_03_TX_CFG, 0x08 | getRFPower());
    mDev.writeReg(CYRF_06_RX_CFG, 0x4A);
    mDev.writeReg(CYRF_0B_PWR_CTRL, 0x00);
    mDev.writeReg(CYRF_10_FRAMING_CFG, 0xA4);
    mDev.writeReg(CYRF_11_DATA32_THOLD, 0x05);
    mDev.writeReg(CYRF_12_DATA64_THOLD, 0x0E);
    mDev.writeReg(CYRF_1B_TX_OFFSET_LSB, 0x55);
    mDev.writeReg(CYRF_1C_TX_OFFSET_MSB, 0x05);
    mDev.writeReg(CYRF_32_AUTO_CAL_TIME, 0x3C);
    mDev.writeReg(CYRF_35_AUTOCAL_OFFSET, 0x14);
    mDev.writeReg(CYRF_39_ANALOG_CTRL, 0x01);
    mDev.writeReg(CYRF_1E_RX_OVERRIDE, 0x10);
    mDev.writeReg(CYRF_1F_TX_OVERRIDE, 0x00);
    mDev.writeReg(CYRF_01_TX_LENGTH, 0x10);
    mDev.writeReg(CYRF_0F_XACT_CFG, 0x10);
    mDev.writeReg(CYRF_27_CLK_OVERRIDE, 0x02);
    mDev.writeReg(CYRF_28_CLK_EN, 0x02);
    mDev.writeReg(CYRF_0F_XACT_CFG, 0x28);
}

void RFProtocolDevo::setRadioChannels(void)
{
    mDev.findBestChannels(mRFChanBufs, 3, 4, 4, 80);

#ifdef BOGUS
    int i;
    printf("Radio Channels:");
    for (i = 0; i < 3; i++) {
        printf(" %02x", mRFChanBufs[i]);
    }
    printf("\n");
#endif /* BOGUS */
    
    //Makes code a little easier to duplicate these here
    mRFChanBufs[3] = mRFChanBufs[0];
    mRFChanBufs[4] = mRFChanBufs[1];
}

void RFProtocolDevo::buildPacket(void)
{
    switch(mState) {
        case DEVO_BIND:
            mBindCtr--;
            buildBindPacket();
            mState = DEVO_BIND_SENDCH;
            break;
        case DEVO_BIND_SENDCH:
            mBindCtr--;
            buildDataPacket();
            buildScramblePacket();
            if (mBindCtr <= 0) {
                mState = DEVO_BOUND;
//                PROTOCOL_SetBindState(0);
            } else {
                mState = DEVO_BIND;
            }
            break;
        case DEVO_BOUND:
        case DEVO_BOUND_1:
        case DEVO_BOUND_2:
        case DEVO_BOUND_3:
        case DEVO_BOUND_4:
        case DEVO_BOUND_5:
        case DEVO_BOUND_6:
        case DEVO_BOUND_7:
        case DEVO_BOUND_8:
        case DEVO_BOUND_9:
            buildDataPacket();
            buildScramblePacket();
            mState++;
            if (mBindCtr > 0) {
                mBindCtr--;
                if (mBindCtr == 0) {
//                    PROTOCOL_SetBindState(0);
                }
            }
            break;
        case DEVO_BOUND_10:
            buildBeaconPacket(mConChanCnt > 8 ? failsafe_pkt : 0);
            failsafe_pkt = failsafe_pkt ? 0 : 1;
            buildScramblePacket();
            mState = DEVO_BOUND_1;
            break;
    }
    mPacketCtr++;
    if(mPacketCtr == PKTS_PER_CHANNEL)
        mPacketCtr = 0;
}

#ifdef BOGUS
u16 RFProtocolDevo::callState2(void)
{
    if (mTxState == 0) {
        mTxState = 1;
        buildPacket();
        mDev.writePayload(mPacketBuf);
        return 900;
    }
    int delay = 100;
    if (mTxState == 1) {
        int i = 0;
        while (! (mDev.readReg(0x04) & 0x02)) {
            if(++i > NUM_WAIT_LOOPS) {
                delay = 1500;
                mTxState = 15;
                break;
            }
        }
     
        if (mState == DEVO_BOUND) {
            /* exit binding mState */
            mState = DEVO_BOUND_3;
            setBoundSOPCodes();
        }
        if(mPacketCtr == 0 || mBindCtr > 0) {
            delay = 1500;
            mTxState = 15;
        } else {
            mDev.setTxRxMode(RX_EN); //Receive mode
            mDev.writeReg(CYRF_07_RX_IRQ_STATUS, 0x80); //Prepare to receive
            mDev.writeReg(CYRF_05_RX_CTRL, 0x80); //Prepare to receive (do not enable any IRQ)
        }
    } else {
        int reg = mDev.readReg(0x07);
        if ((reg & 0x23) == 0x22)
        //if(mDev.readReg(0x07) & 0x20)
        { // this won't be true in emulator so we need to simulate it somehow
            mDev.readPayload(mPacketBuf);
            parseTelemetryPacket(mPacketBuf);
            delay = 100 * (16 - mTxState);
            mTxState = 15;
        }
#ifdef EMULATOR
        u8 telem_bit = rand32() % 7; // random number in [0, 7)
        mPacketBuf[0] =  TELEMETRY_ENABLE + telem_bit; // allow emulator to simulate telemetry parsing to prevent future bugs in the telemetry monitor
        //printf("telem 1st mPacketBuf: 0x%x\n", mPacketBuf[0]);
        for(int i = 1; i < 13; i++)
            mPacketBuf[i] = rand32() % 256;
        parseTelemetryPacket(mPacketBuf);
        for(int i = 0; i < TELEM_UPDATE_SIZE; i++)
            Telemetry.updated[i] = 0xff;
        delay = 100 * (16 - mTxState);
        mTxState = 15;
#endif
    }
    mTxState++;
    if(mTxState == 16) { //2.3msec have passed
        mDev.setTxRxMode(TX_EN); //Write mode
        if(mPacketCtr == 0) {
            //Keep tx power updated
            mDev.writeReg(CYRF_03_TX_CFG, 0x08 | Model.tx_power);
            radio_ch_ptr = radio_ch_ptr == &mRFChanBufs[2] ? mRFChanBufs : radio_ch_ptr + 1;
            CYRF_ConfigRFChannel(*radio_ch_ptr);
        }
        mTxState = 0;
    }
    return PACKET_PERIOD_MS;
}
#endif /* BOGUS */

#define NUM_WAIT_LOOPS (100 / 5) //each loop is ~5us.  Do not wait more than 100us

u16 RFProtocolDevo::callState(void)
{
    int i = 0;
    
    if (mTxState == 0) {
        mTxState = 1;
        buildPacket();
        mDev.writePayload(mPacketBuf, MAX_PACKET_SIZE);
        return PACKET_PERIOD_MS;
    }
    
    mTxState = 0;
    while (! (mDev.readReg(0x04) & 0x02)) {
        if(++i > NUM_WAIT_LOOPS)
            return PACKET_PERIOD_MS;
    }
    if (mState == DEVO_BOUND) {
        /* exit binding mState */
        mState = DEVO_BOUND_3;
        setBoundSOPCodes();
    }
    if(mPacketCtr == 0) {
        //Keep tx power updated
        mDev.writeReg(CYRF_03_TX_CFG, 0x08 | getRFPower());
        radio_ch_ptr = radio_ch_ptr == &mRFChanBufs[2] ? mRFChanBufs : radio_ch_ptr + 1;
        mDev.setRFChannel(*radio_ch_ptr);
    }
    return PACKET_PERIOD_MS;
}

void RFProtocolDevo::test(s8 id)
{
}

int RFProtocolDevo::init(void)
{
    init1();

    use_fixed_id = 0;
    failsafe_pkt = 0;
    radio_ch_ptr = mRFChanBufs;
//    memset(&Telemetry, 0, sizeof(Telemetry));

    mDev.setRFChannel(*radio_ch_ptr);
    mPacketCtr = 0;
    mConChanIdx = 0;
    mTxState = 0;

    if(1) {  // ! Model.fixed_id
        fixed_id = ((u32)(mRFChanBufs[0] ^ mMFGID[0] ^ mMFGID[3]) << 16)
                 | ((u32)(mRFChanBufs[1] ^ mMFGID[1] ^ mMFGID[4]) << 8)
                 | ((u32)(mRFChanBufs[2] ^ mMFGID[2] ^ mMFGID[5]) << 0);
        fixed_id = fixed_id % 1000000;
        mBindCtr = MAX_BIND_COUNT;
        mState   = DEVO_BIND;
        //PROTOCOL_SetBindState(0x1388 * 2400 / 1000); //msecs
    } else {
        //fixed_id = Model.fixed_id;
        use_fixed_id = 1;
        mState = DEVO_BOUND_1;
        mBindCtr = 0;
        setBoundSOPCodes();
    }

    startState(INITIAL_WAIT_MS);
    printf(F("init : %ld\n"), millis());
    return 0;
}

int RFProtocolDevo::close(void)
{
    //printf(F("%08ld : %s\n"), millis(), __PRETTY_FUNCTION__);
    
    RFProtocol::close();
    mDev.initialize();
    return (mDev.reset() ? 1L : -1L);
}

int RFProtocolDevo::reset(void)
{
    return close();
}

int RFProtocolDevo::getChannels(void)
{
    return 6;
}

int RFProtocolDevo::getInfo(s8 id, u8 *data)
{
    u8 size = 0;
    switch (id) {
        case INFO_STATE:
            *data = mState;
            size = 1;
            break;

        case INFO_CHANNEL:
            *data = mRFChanBufs[0];
            size = 1;
            break;

        case INFO_PACKET_CTR:
            size = sizeof(mPacketCtr);
            *((u32*)data) = mPacketCtr;
            break;

        case INFO_ID:
            size = 4;
            *((u32*)data) = (u32)getProtoID();
            break;
    }
    return size;
}
