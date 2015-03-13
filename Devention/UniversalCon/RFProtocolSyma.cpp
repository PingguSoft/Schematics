#include <SPI.h>
#include "RFProtocolSyma.h"
#include "utils.h"

u8 RFProtocolSyma::getCheckSum(u8 *data)
{
    u8 sum = data[0];

    for (int i=1; i < mPacketSize-1; i++)
        if (getProtoOpt() == FORMAT_X5C_X2)
            sum += data[i];
        else
            sum ^= data[i];

    return sum + ((getProtoOpt() == FORMAT_X5C_X2) ? 0 : 0x55);
}

u8 RFProtocolSyma::checkStatus()
{
    u8 stat = mDev.readReg(NRF24L01_07_STATUS);
    printf(F("checkStatus :%x\n"), stat);
}


#define BABS(X) (((X) < 0) ? -(u8)(X) : (X))
u8 RFProtocolSyma::getChannel(CH_T id)
{
    s32 ch = RFProtocol::getControl(id);
    if (ch < CHAN_MIN_VALUE) {
        ch = CHAN_MIN_VALUE;
    } else if (ch > CHAN_MAX_VALUE) {
        ch = CHAN_MAX_VALUE;
    }

    u8 ret = (u8) ((ch < 0 ? 0x80 : 0) | BABS(ch * 127 / CHAN_MAX_VALUE));
    return ret;
}

#define X5C_CHAN2TRIM(X) ((((X) & 0x80 ? 0xff - (X) : 0x80 + (X)) >> 2) + 0x20)
void RFProtocolSyma::buildPacketX5C(u8 bind)
{
    if (bind) {
        memset(mPacketBuf, 0, mPacketSize);
        mPacketBuf[7] = 0xae;
        mPacketBuf[8] = 0xa9;
        mPacketBuf[14] = 0xc0;
        mPacketBuf[15] = 0x17;
    } else {
        mPacketBuf[0] = getChannel(CH_THROTTLE);
        mPacketBuf[0] = (mPacketBuf[0] & 0x80) ? (0xff - mPacketBuf[0]) : (0x80 + mPacketBuf[0]);
        mPacketBuf[1] = getChannel(CH_RUDDER);
        mPacketBuf[2] = getChannel(CH_ELEVATOR) ^ 0x80;  // reversed from default
        mPacketBuf[3] = getChannel(CH_AILERON);
        mPacketBuf[4] = X5C_CHAN2TRIM(getChannel(CH_RUDDER) ^ 0x80);     // drive trims for extra control range
        mPacketBuf[5] = X5C_CHAN2TRIM(getChannel(CH_ELEVATOR));
        mPacketBuf[6] = X5C_CHAN2TRIM(getChannel(CH_AILERON) ^ 0x80);
        mPacketBuf[7] = 0xae;
        mPacketBuf[8] = 0xa9;
        mPacketBuf[9] = 0x00;
        mPacketBuf[10] = 0x00;
        mPacketBuf[11] = 0x00;
        mPacketBuf[12] = 0x00;
        mPacketBuf[13] = 0x00;
        mPacketBuf[14] = (mAuxFlag & FLAG_VIDEO   ? 0x10 : 0x00)
                   | (mAuxFlag & FLAG_PICTURE ? 0x08 : 0x00)
                   | (mAuxFlag & FLAG_FLIP    ? 0x01 : 0x00)
                   | 0x04;  // always high rates (bit 3 is rate control)
        mPacketBuf[15] = getCheckSum(mPacketBuf);
    }
}

void RFProtocolSyma::buildPacket(u8 bind)
{
    if (bind) {
        mPacketBuf[0] = mRxTxAddrBuf[4];
        mPacketBuf[1] = mRxTxAddrBuf[3];
        mPacketBuf[2] = mRxTxAddrBuf[2];
        mPacketBuf[3] = mRxTxAddrBuf[1];
        mPacketBuf[4] = mRxTxAddrBuf[0];
        mPacketBuf[5] = 0xaa;
        mPacketBuf[6] = 0xaa;
        mPacketBuf[7] = 0xaa;
        mPacketBuf[8] = 0x00;
    } else {
        mPacketBuf[0] = getChannel(CH_THROTTLE);
        mPacketBuf[0] = (mPacketBuf[0] & 0x80) ? (0xff - mPacketBuf[0]) : (0x80 + mPacketBuf[0]);
        mPacketBuf[1] = getChannel(CH_ELEVATOR);
        mPacketBuf[2] = getChannel(CH_RUDDER);
        mPacketBuf[3] = getChannel(CH_AILERON);
        mPacketBuf[4] = (mAuxFlag & FLAG_VIDEO   ? 0x80 : 0x00)
                  | (mAuxFlag & FLAG_PICTURE ? 0x40 : 0x00);
        // use trims to extend controls
        mPacketBuf[5] = (getChannel(CH_ELEVATOR) >> 2) | 0xc0; // always high rates (bit 7 is rate control)
        mPacketBuf[6] = (getChannel(CH_RUDDER) >> 2)   | (mAuxFlag & FLAG_FLIP  ? 0x40 : 0x00);
        mPacketBuf[7] = getChannel(CH_AILERON) >> 2;
        mPacketBuf[8] = 0x00;
    }
    mPacketBuf[9] = getCheckSum(mPacketBuf);

}

void RFProtocolSyma::sendPacket(u8 bind)
{
    if (getProtoOpt() == FORMAT_X5C_X2)
      buildPacketX5C(bind);
    else
      buildPacket(bind);

    // clear mPacketBuf status bits and TX FIFO
    mDev.writeReg(NRF24L01_07_STATUS, 0x70);
    mDev.writeReg(NRF24L01_00_CONFIG, 0x2e);
    mDev.writeReg(NRF24L01_05_RF_CH, mChannelBuf[mCurChan]);
    mDev.flushTx();
    mDev.writePayload(mPacketBuf, mPacketSize);


    if (mPacketCtr++ % 2) {   // use each channel twice
        mCurChan = (mCurChan + 1) % mChannelCnt;
    }

//    printf(F("SEND PACKET bind:%d :%d\n"), bind, mPacketCtr);

//    radio.ce(HIGH);
//    delayMicroseconds(15);
    // It saves power to turn off radio after the transmission,
    // so as long as we have pins to do so, it is wise to turn
    // it back.
//    radio.ce(LOW);

    // Check and adjust transmission power. We do this after
    // transmission to not bother with timeout after power
    // settings change -  we have plenty of time until next
    // mPacketBuf.
//    if (tx_power != Model.tx_power) {
        //Keep transmit power updated
//        tx_power = Model.tx_power;
//        NRF24L01_SetPower(tx_power);
//    }

//    printf(F("SEND PACKET 2 : %x\n"), mDev.readReg(NRF24L01_17_FIFO_STATUS));

#if 0
    u8 stat;

    do {
      stat = mDev.readReg(NRF24L01_07_STATUS);
//      printf(F("SEND PACKET bind:%d : %x\n"), bind, stat);
    } while ((stat & BV(5)) == 0);
    mDev.writeReg(NRF24L01_07_STATUS, BV(5));

//    frameloss += mDev.readReg(NRF24L01_08_OBSERVE_TX) >> 4;
//    printf(F("SEND PACKET loss:%d\n"), frameloss);
#endif
}


void RFProtocolSyma::initRxTxAddr(void)
{
    u32 lfsr = getControllerID();

    // Pump zero bytes for LFSR to diverge more
    for (u8 i = 0; i < sizeof(lfsr); ++i)
      rand32_r(&lfsr, 0);

    mRxTxAddrBuf[4] = 0xa2;
    for (u8 i = 0; i < sizeof(mRxTxAddrBuf)-1; ++i) {
        mRxTxAddrBuf[i] = lfsr & 0xff;
        rand32_r(&lfsr, i);
    }

    printf(F("ID:%08lx\n"), lfsr);
}

void RFProtocolSyma::init1(void)
{
    const u8 bind_rx_tx_addr[] = {0xab,0xac,0xad,0xae,0xaf};
    const u8 rx_tx_addr_x5c[]  = {0x6d,0x6a,0x73,0x73,0x73};    // X5C uses same address for bind and data

    mDev.initialize();

    mDev.setTxRxMode(TX_EN);

    mDev.readReg(NRF24L01_07_STATUS);
    mDev.writeReg(NRF24L01_00_CONFIG, BV(NRF24L01_00_EN_CRC) | BV(NRF24L01_00_CRCO));
    mDev.writeReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknoledgement
    mDev.writeReg(NRF24L01_02_EN_RXADDR, 0x3F);  // Enable all data pipes (even though not used?)
    mDev.writeReg(NRF24L01_03_SETUP_AW, 0x03);   // 5-byte RX/TX address
    mDev.writeReg(NRF24L01_04_SETUP_RETR, 0xff); // 4mS retransmit t/o, 15 tries (retries w/o AA?)
    mDev.writeReg(NRF24L01_05_RF_CH, 0x08);

    if (getProtoOpt() == FORMAT_X5C_X2) {
      mDev.setBitrate(NRF24L01_BR_1M);
      mPacketSize = 16;
    } else {
      mDev.setBitrate(NRF24L01_BR_250K);
      mPacketSize = 10;
    }

    mDev.setPower(TXPOWER_100mW);
    mDev.writeReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
    mDev.writeReg(NRF24L01_08_OBSERVE_TX, 0x00);
    mDev.writeReg(NRF24L01_09_CD, 0x00);
    mDev.writeReg(NRF24L01_0C_RX_ADDR_P2, 0xC3); // LSB byte of pipe 2 receive address
    mDev.writeReg(NRF24L01_0D_RX_ADDR_P3, 0xC4);
    mDev.writeReg(NRF24L01_0E_RX_ADDR_P4, 0xC5);
    mDev.writeReg(NRF24L01_0F_RX_ADDR_P5, 0xC6);
    mDev.writeReg(NRF24L01_11_RX_PW_P0, PAYLOADSIZE);   // bytes of data payload for pipe 1
    mDev.writeReg(NRF24L01_12_RX_PW_P1, PAYLOADSIZE);
    mDev.writeReg(NRF24L01_13_RX_PW_P2, PAYLOADSIZE);
    mDev.writeReg(NRF24L01_14_RX_PW_P3, PAYLOADSIZE);
    mDev.writeReg(NRF24L01_15_RX_PW_P4, PAYLOADSIZE);
    mDev.writeReg(NRF24L01_16_RX_PW_P5, PAYLOADSIZE);
    mDev.writeReg(NRF24L01_17_FIFO_STATUS, 0x00); // Just in case, no real bits to write here

    mDev.writeRegisterMulti(NRF24L01_10_TX_ADDR,
                                (getProtoOpt() == FORMAT_X5C_X2) ? rx_tx_addr_x5c : bind_rx_tx_addr,
                                5);

    mDev.readReg(NRF24L01_07_STATUS);

#if 0
    // Check for Beken BK2421/BK2423 chip
    // It is done by using Beken specific activate code, 0x53
    // and checking that status register changed appropriately
    // There is no harm to run it on nRF24L01 because following
    // closing activate command changes state back even if it
    // does something on nRF24L01
    NRF24L01_Activate(0x53); // magic for BK2421 bank switch
    dbgprintf("Trying to switch banks\n");
    if (mDev.readReg(NRF24L01_07_STATUS) & 0x80) {
        dbgprintf("BK2421 detected\n");
        // Beken registers don't have such nice names, so we just mention
        // them by their numbers
        // It's all magic, eavesdropped from real transfer and not even from the
        // data sheet - it has slightly different values
        mDev.writeRegisterMulti(0x00, (u8 *) "\x40\x4B\x01\xE2", 4);
        mDev.writeRegisterMulti(0x01, (u8 *) "\xC0\x4B\x00\x00", 4);
        mDev.writeRegisterMulti(0x02, (u8 *) "\xD0\xFC\x8C\x02", 4);
        mDev.writeRegisterMulti(0x03, (u8 *) "\x99\x00\x39\x21", 4);
        mDev.writeRegisterMulti(0x04, (u8 *) "\xF9\x96\x82\x1B", 4);
        mDev.writeRegisterMulti(0x05, (u8 *) "\x24\x06\x7F\xA6", 4);
        mDev.writeRegisterMulti(0x06, (u8 *) "\x00\x00\x00\x00", 4);
        mDev.writeRegisterMulti(0x07, (u8 *) "\x00\x00\x00\x00", 4);
        mDev.writeRegisterMulti(0x08, (u8 *) "\x00\x00\x00\x00", 4);
        mDev.writeRegisterMulti(0x09, (u8 *) "\x00\x00\x00\x00", 4);
        mDev.writeRegisterMulti(0x0A, (u8 *) "\x00\x00\x00\x00", 4);
        mDev.writeRegisterMulti(0x0B, (u8 *) "\x00\x00\x00\x00", 4);
        mDev.writeRegisterMulti(0x0C, (u8 *) "\x00\x12\x73\x00", 4);
        mDev.writeRegisterMulti(0x0D, (u8 *) "\x46\xB4\x80\x00", 4);
        mDev.writeRegisterMulti(0x0E, (u8 *) "\x41\x10\x04\x82\x20\x08\x08\xF2\x7D\xEF\xFF", 11);
        mDev.writeRegisterMulti(0x04, (u8 *) "\xFF\x96\x82\x1B", 4);
        mDev.writeRegisterMulti(0x04, (u8 *) "\xF9\x96\x82\x1B", 4);
    } else {
        dbgprintf("nRF24L01 detected\n");
    }
    NRF24L01_Activate(0x53); // switch bank back
#endif

    mDev.flushTx();
    mDev.readReg(NRF24L01_07_STATUS);
    mDev.writeReg(NRF24L01_07_STATUS, 0x0e);
    mDev.readReg(NRF24L01_00_CONFIG);
    mDev.writeReg(NRF24L01_00_CONFIG, 0x0c);
    mDev.writeReg(NRF24L01_00_CONFIG, 0x0e);  // power on

    printf(F("init1 : %ld\n"), millis());
}



// write a strange first mPacketBuf to RF channel 8 ...
const PROGMEM u8 first_packet[]   = { 0xf9, 0x96, 0x82, 0x1b, 0x20, 0x08, 0x08, 0xf2,
                                      0x7d, 0xef, 0xff, 0x00, 0x00, 0x00, 0x00 };
const PROGMEM u8 chans_bind[]     = { 0x4b, 0x30, 0x40, 0x2e };
const PROGMEM u8 chans_bind_x5c[] = { 0x27, 0x1b, 0x39, 0x28, 0x24, 0x22, 0x2e, 0x36,
                                      0x19, 0x21, 0x29, 0x14, 0x1e, 0x12, 0x2d, 0x18};

void RFProtocolSyma::init2(void)
{
    mDev.flushTx();
    mDev.writeReg(NRF24L01_05_RF_CH, 0x08);
    memcpy_P(mChannelBuf, first_packet, sizeof(first_packet));
    mDev.writePayload(mChannelBuf, sizeof(first_packet));

    if (getProtoOpt() == FORMAT_X5C_X2) {
      mChannelCnt = sizeof(chans_bind_x5c);
      memcpy_P(mChannelBuf, chans_bind_x5c, mChannelCnt);
    } else {
      initRxTxAddr();   // make info available for bind packets
      mChannelCnt = sizeof(chans_bind);
      memcpy_P(mChannelBuf, chans_bind, mChannelCnt);
    }

    mCurChan   = 0;
    mPacketCtr = 0;
    printf(F("init2 : %ld\n"), millis());
}

const PROGMEM u8 chans_data_x5c[] = {0x1d, 0x2f, 0x26, 0x3d, 0x15, 0x2b, 0x25, 0x24,
                                     0x27, 0x2c, 0x1c, 0x3e, 0x39, 0x2d, 0x22};
void RFProtocolSyma::init3(void)
{
    if (getProtoOpt() == FORMAT_X5C_X2) {
      mChannelCnt = sizeof(chans_data_x5c);
      memcpy_P(mChannelBuf, chans_data_x5c, mChannelCnt);
    } else {
      setRFChannel(mRxTxAddrBuf[0]);
      mDev.writeRegisterMulti(NRF24L01_10_TX_ADDR, mRxTxAddrBuf, 5);
    }
    mCurChan   = 0;
    mPacketCtr = 0;
    printf(F("init3 : %ld\n"), millis());
}

const PROGMEM u8 start_chans_1[] = {0x0a, 0x1a, 0x2a, 0x3a};
const PROGMEM u8 start_chans_2[] = {0x2a, 0x0a, 0x42, 0x22};
const PROGMEM u8 start_chans_3[] = {0x1a, 0x3a, 0x12, 0x32};

// channels determined by last byte of tx address
void RFProtocolSyma::setRFChannel(u8 address)
{
  u8 laddress = address & 0x1f;
  u8 i;
  u32 *pchans = (u32 *)mChannelBuf;   // avoid compiler warning

  mChannelCnt = 4;
  if (laddress < 0x10) {
    if (laddress == 6) laddress = 7;
    for(i=0; i < mChannelCnt; i++) {
      mChannelBuf[i] = pgm_read_byte(start_chans_1 + i) + laddress;
    }
  } else if (laddress < 0x18) {
    for(i=0; i < mChannelCnt; i++) {
      mChannelBuf[i] = pgm_read_byte(start_chans_2 + i) + (laddress & 0x07);
    }
    if (laddress == 0x16) {
      mChannelBuf[0] += 1;
      mChannelBuf[1] += 1;
    }
  } else if (laddress < 0x1e) {
    for(i=0; i < mChannelCnt; i++) {
      mChannelBuf[i] = pgm_read_byte(start_chans_3 + i) + (laddress & 0x07);
    }
  } else if (laddress == 0x1e) {
      *pchans = 0x38184121;
  } else {
      *pchans = 0x39194121;
  }
}

u16 RFProtocolSyma::callState(void)
{
    switch (mState) {
    case SYMAX_INIT1:
        init2();
        mState = SYMAX_BIND2;
        return FIRST_PACKET_MS;
        break;

    case SYMAX_BIND2:
        mBindCtr = MAX_BIND_COUNT;
        mState   = SYMAX_BIND3;
        sendPacket(1);
        break;

    case SYMAX_BIND3:
        if (mBindCtr == 0) {
            init3();
            mState = SYMAX_DATA;
            printf(F("Bind Done : %ld\n"), millis());
        } else {
            sendPacket(1);
            mBindCtr--;
        }
        break;

    case SYMAX_DATA:
        sendPacket(0);
        break;
    }
    return PACKET_PERIOD_MS;
}

void RFProtocolSyma::test(s8 id)
{
}

void RFProtocolSyma::handleTimer(s8 id)
{
    if (id == mTmrState) {
        u16 time = callState();
        mTmrState = after(time);
    }
}

void RFProtocolSyma::loop(void)
{
    update();
}

int RFProtocolSyma::init(void)
{
    mPacketCtr = 0;
    mAuxFlag   = 0;
    mTmrState  = -1;

    init1();
    mState = SYMAX_INIT1;

    mTmrState = after(INITIAL_WAIT_MS);
    printf(F("init : %ld\n"), millis());
    return 0;
}

int RFProtocolSyma::close(void)
{
    //printf(F("%08ld : %s\n"), millis(), __PRETTY_FUNCTION__);
    mDev.initialize();
    return (mDev.reset() ? 1L : -1L);
}

int RFProtocolSyma::reset(void)
{
    return close();
}

int RFProtocolSyma::getChannels(void)
{
    return 6;
}

int RFProtocolSyma::setPower(int power)
{
    mDev.setPower(power);
    return 0;
}

int RFProtocolSyma::getInfo(s8 id, u8 *data, u8 *size)
{
    switch (id) {
        case INFO_STATE:
            *data = mState;
            *size = 1;
            break;

        case INFO_CHANNEL:
            *data = mChannelBuf[mCurChan];
            *size = 1;
            break;

        case INFO_PACKET_CTR:
            *size = sizeof(mPacketCtr);
            *((u32*)data) = mPacketCtr;
            break;
    }
}

