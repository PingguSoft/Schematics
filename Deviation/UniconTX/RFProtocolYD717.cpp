#include <SPI.h>
#include "RFProtocolYD717.h"
#include "utils.h"


u8 RFProtocolYD717::checkStatus()
{
    u8 stat = mDev.readReg(NRF24L01_07_STATUS);

//    printf(F("checkStatus :%x\n"), stat);
    switch (stat & (BV(NRF24L01_07_TX_DS) | BV(NRF24L01_07_MAX_RT))) 
    {
    case BV(NRF24L01_07_TX_DS):
        return PKT_ACKED;
    case BV(NRF24L01_07_MAX_RT):
        return PKT_TIMEOUT;
    }
    
    return PKT_PENDING;
}

u8 RFProtocolYD717::getControl(CH_T id)
{
    s32 ch = RFProtocol::getControl(id);
    if (ch < CHAN_MIN_VALUE) {
        ch = CHAN_MIN_VALUE;
    } else if (ch > CHAN_MAX_VALUE) {
        ch = CHAN_MAX_VALUE;
    }

    u8 ret = (u8) (((ch * 0xFF / CHAN_MAX_VALUE) + 0x100) >> 1);
    return ret;
}

void RFProtocolYD717::getControls(u8* throttle, u8* rudder, u8* elevator, u8* aileron,
                          u8* flags, u8* rudder_trim, u8* elevator_trim, u8* aileron_trim)
{
    // RFProtocol is registered AETRF, that is
    // Aileron is channel 1, Elevator - 2, Throttle - 3, Rudder - 4, Flip control - 5

    // Channel 3
    *throttle = getControl(CH_THROTTLE);

    // Channel 4
    if(getProtocolOpt() == FORMAT_XINXUN) {
      *rudder = getControl(CH_RUDDER);
      *rudder_trim = (0xff - *rudder) >> 1;
    } else {
      *rudder = 0xff - getControl(CH_RUDDER);
      *rudder_trim = *rudder >> 1;
    }

    // Channel 2
    *elevator = getControl(CH_ELEVATOR);
    *elevator_trim = *elevator >> 1;

    // Channel 1
    *aileron = 0xff - getControl(CH_AILERON);
    *aileron_trim = *aileron >> 1;

    // Channel 5
    if (RFProtocol::getControl(CH_AUX1) <= 0)
      *flags &= ~YD717_FLAG_FLIP;
    else
      *flags |= YD717_FLAG_FLIP;

    // Channel 6
    if (RFProtocol::getControl(CH_AUX2) <= 0)
      *flags &= ~YD717_FLAG_LIGHT;
    else
      *flags |= YD717_FLAG_LIGHT;


//    printf(F("ail %3d+%3d, ele %3d+%3d, thr %3d, rud %3d+%3d, flip enable %d\n"),
//            *aileron, *aileron_trim, *elevator, *elevator_trim, *throttle,
//            *rudder, *rudder_trim, *flags);
}

void RFProtocolYD717::sendPacket(u8 bind)
{
    if (bind) {
        mPacketBuf[0]= mRxTxAddrBuf[0]; // send data phase address in first 4 bytes
        mPacketBuf[1]= mRxTxAddrBuf[1];
        mPacketBuf[2]= mRxTxAddrBuf[2];
        mPacketBuf[3]= mRxTxAddrBuf[3];
        mPacketBuf[4] = 0x56;
        mPacketBuf[5] = 0xAA;
        mPacketBuf[6] = (getProtocolOpt() == FORMAT_NI_HUI) ? 0x00 : 0x32;
        mPacketBuf[7] = 0x00;
    } else {
        if (getProtocolOpt() == FORMAT_YD717)
            getControls(&mPacketBuf[0], &mPacketBuf[1], &mPacketBuf[3], &mPacketBuf[4], &mPacketBuf[7], &mPacketBuf[6], &mPacketBuf[2], &mPacketBuf[5]);
        else
            getControls(&mPacketBuf[0], &mPacketBuf[1], &mPacketBuf[3], &mPacketBuf[4], &mPacketBuf[7], &mPacketBuf[2], &mPacketBuf[5], &mPacketBuf[6]);
    }

    // clear mPacketBuf status bits and TX FIFO
    mDev.writeReg(NRF24L01_07_STATUS, (BV(NRF24L01_07_TX_DS) | BV(NRF24L01_07_MAX_RT)));

    if(getProtocolOpt() == FORMAT_YD717) {
        mDev.writePayload(mPacketBuf, 8);
    } else {
        mPacketBuf[8] = mPacketBuf[0];  // checksum
        for(u8 i=1; i < 8; i++) mPacketBuf[8] += mPacketBuf[i];
        mPacketBuf[8] = ~mPacketBuf[8];
        mDev.writePayload(mPacketBuf, 9);
    }
    ++mPacketCtr;

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
}


void RFProtocolYD717::initRxTxAddr(void)
{
    u32 lfsr = getControllerID();

    // Pump zero bytes for LFSR to diverge more
    for (u8 i = 0; i < sizeof(lfsr); ++i)
      rand32_r(&lfsr, 0);

    mRxTxAddrBuf[4] = 0xC1;
    for (u8 i = 0; i < sizeof(mRxTxAddrBuf)-1; ++i) {
        mRxTxAddrBuf[i] = lfsr & 0xff;
        rand32_r(&lfsr, i);
    }
    printf(F("ID:%08lx\n"), lfsr);    
}

void RFProtocolYD717::init1(void)
{
    mDev.initialize();

    // CRC, radio on
    mDev.setTxRxMode(TX_EN);

    mDev.writeReg(NRF24L01_00_CONFIG, BV(NRF24L01_00_EN_CRC) | BV(NRF24L01_00_PWR_UP)); 
    mDev.writeReg(NRF24L01_01_EN_AA, 0x3F);      // Auto Acknoledgement on all data pipes
    mDev.writeReg(NRF24L01_02_EN_RXADDR, 0x3F);  // Enable all data pipes
    mDev.writeReg(NRF24L01_03_SETUP_AW, 0x03);   // 5-byte RX/TX address
    mDev.writeReg(NRF24L01_04_SETUP_RETR, 0x1A); // 500uS retransmit t/o, 10 tries
    mDev.writeReg(NRF24L01_05_RF_CH, RF_CHANNEL);      // Channel 3C
    mDev.setBitrate(NRF24L01_BR_1M);             // 1Mbps
    mDev.setPower(TXPOWER_100mW);
    mDev.writeReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
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
    mDev.writeReg(NRF24L01_1C_DYNPD, 0x3F);       // Enable dynamic payload length on all pipes

    // this sequence necessary for module from stock tx
    mDev.readReg(NRF24L01_1D_FEATURE);
    mDev.activate(0x73);                          // Activate feature register
    mDev.readReg(NRF24L01_1D_FEATURE);
    mDev.writeReg(NRF24L01_1C_DYNPD, 0x3F);       // Enable dynamic payload length on all pipes
    mDev.writeReg(NRF24L01_1D_FEATURE, 0x07);     // Set feature bits on

    mDev.writeRegisterMulti(NRF24L01_0A_RX_ADDR_P0, mRxTxAddrBuf, 5);
    mDev.writeRegisterMulti(NRF24L01_10_TX_ADDR, mRxTxAddrBuf, 5);

    printf(F("init1 : %ld\n"), millis());
}

void RFProtocolYD717::init2(void)
{
    // for bind packets set address to prearranged value known to receiver
    u8 bind_rx_tx_addr[5];
    
    if (getProtocolOpt() == FORMAT_SYMA_X4)
        for(u8 i=0; i < 5; i++) bind_rx_tx_addr[i]  = 0x60;
    else if (getProtocolOpt() == FORMAT_NI_HUI)
        for(u8 i=0; i < 5; i++) bind_rx_tx_addr[i]  = 0x64;
    else
        for(u8 i=0; i < 5; i++) bind_rx_tx_addr[i]  = 0x65;

    mDev.writeRegisterMulti(NRF24L01_0A_RX_ADDR_P0, bind_rx_tx_addr, 5);
    mDev.writeRegisterMulti(NRF24L01_10_TX_ADDR, bind_rx_tx_addr, 5);
    printf(F("init2 : %ld\n"), millis());
}

void RFProtocolYD717::init3(void)
{
    // set rx/tx address for data phase
    mDev.writeRegisterMulti(NRF24L01_0A_RX_ADDR_P0, mRxTxAddrBuf, 5);
    mDev.writeRegisterMulti(NRF24L01_10_TX_ADDR, mRxTxAddrBuf, 5);
    printf(F("init3 : %ld\n"), millis());
}

#ifdef YD717_TELEMETRY
void RFProtocolYD717::updateTelemetry(void) {
  static u8 frameloss = 0;

  frameloss += mDev.ReadReg(NRF24L01_08_OBSERVE_TX) >> 4;
  mDev.writeReg(NRF24L01_05_RF_CH, RF_CHANNEL);   // reset packet loss counter
}
#endif

u16 RFProtocolYD717::callState(void)
{
    switch (mState) {
    case YD717_INIT1:
        sendPacket(0);
        mState = YD717_BIND3;
        break;

    case YD717_BIND2:
        if (mBindCtr == 0) {
            if (checkStatus() == PKT_PENDING)
                return PACKET_CHKTIME_MS;       // packet send not yet complete
            init3();                            // change to data phase rx/tx address
            sendPacket(0);
            mState = YD717_BIND3;
        } else {
            if (checkStatus() == PKT_PENDING)
                return PACKET_CHKTIME_MS;       // packet send not yet complete
            sendPacket(1);
            mBindCtr--;
        }
        break;

    case YD717_BIND3:
        switch (checkStatus()) {
        case PKT_PENDING:
            return PACKET_CHKTIME_MS;           // packet send not yet complete
        case PKT_ACKED:
            mState = YD717_DATA;
            printf(F("Bind Done : %ld\n"), millis());
            break;
        case PKT_TIMEOUT:
            init2();                            // change to bind rx/tx address
            mBindCtr = MAX_BIND_COUNT;
            mState = YD717_BIND2;
            sendPacket(1);
        }
        break;

    case YD717_DATA:

#ifdef YD717_TELEMETRY
        update_telemetry();
#endif
        if (checkStatus() == PKT_PENDING)
            return PACKET_CHKTIME_MS;           // packet send not yet complete

        sendPacket(0);
        break;
    }
    return PACKET_PERIOD_MS;
}

void RFProtocolYD717::test(s8 id)
{
}

void RFProtocolYD717::handleTimer(s8 id)
{
    if (id == mTmrState) {
        u16 time = callState();
        mTmrState = after(time);
    }
}

void RFProtocolYD717::loop(void)
{
    update();
}

int RFProtocolYD717::init(void)
{
    mPacketCtr = 0;
    mAuxFlag   = 0;
    mTmrState  = -1;

    initRxTxAddr();
    init1();
    mState = YD717_INIT1;

    mTmrState = after(INITIAL_WAIT_MS);
    return 0;
}

int RFProtocolYD717::close(void)
{
    //printf(F("%08ld : %s\n"), millis(), __PRETTY_FUNCTION__);
    mDev.initialize();
    return (mDev.reset() ? 1L : -1L);
}

int RFProtocolYD717::reset(void)
{
    return close();
}

int RFProtocolYD717::getChannels(void)
{
    return 6;
}

int RFProtocolYD717::setPower(int power)
{
    mDev.setPower(power);
    return 0;
}

int RFProtocolYD717::getInfo(s8 id, u8 *data)
{
    u8 size = 0;
    switch (id) {
        case INFO_STATE:
            *data = mState;
            size = 1;
            break;

        case INFO_CHANNEL:
            *data = RF_CHANNEL;
            size = 1;
            break;

        case INFO_PACKET_CTR:
            size = sizeof(mPacketCtr);
            *((u32*)data) = mPacketCtr;
            break;
    }
    return size;
}