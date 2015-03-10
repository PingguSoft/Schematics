#include "common.h"
#include "interface.h"
#include "ProtocolYD717.h"

u8 ProtocolYD717::ackPacket()
{
    switch (NRF24L01_ReadReg(NRF24L01_07_STATUS) & (BV(NRF24L01_07_TX_DS) | BV(NRF24L01_07_MAX_RT))) {
    case BV(NRF24L01_07_TX_DS):
        return PKT_ACKED;
    case BV(NRF24L01_07_MAX_RT):
        return PKT_TIMEOUT;
    }
    return PKT_PENDING;
}


void ProtocolYD717::sendPacket(u8 bind)
{
    if (bind) {
        packet[0]= rx_tx_addr[0]; // send data phase address in first 4 bytes
        packet[1]= rx_tx_addr[1];
        packet[2]= rx_tx_addr[2];
        packet[3]= rx_tx_addr[3];
        packet[4] = 0x56;
        packet[5] = 0xAA;
        packet[6] = 0x32;
        packet[7] = 0x00;
    } else {
/*        read_controls(&throttle, &rudder, &elevator, &aileron, &flags, &rudder_trim, &elevator_trim, &aileron_trim);
        packet[0] = throttle;
        packet[1] = rudder;
        packet[3] = elevator;
        packet[4] = aileron;
        
        if(Model.protocol == PROTOCOL_YD717 && Model.proto_opts[PROTOOPTS_FORMAT] == FORMAT_YD717) {
            packet[2] = elevator_trim;
            packet[5] = aileron_trim;
            packet[6] = rudder_trim;
        } else 
        {
            packet[2] = rudder_trim;
            packet[5] = elevator_trim;
            packet[6] = aileron_trim;
        }
        packet[7] = flags;
*/        
    }


    // clear packet status bits and TX FIFO
    NRF24L01_WriteReg(NRF24L01_07_STATUS, (BV(NRF24L01_07_TX_DS) | BV(NRF24L01_07_MAX_RT)));
    NRF24L01_FlushTx();

//    if(Model.protocol == PROTOCOL_YD717 && Model.proto_opts[PROTOOPTS_FORMAT] == FORMAT_YD717) {
//        NRF24L01_WritePayload(packet, 8);
//    } else {
    {
        packet[8] = packet[0];  // checksum
        for(u8 i=1; i < 8; i++) packet[8] += packet[i];
        packet[8] = ~packet[8];

        NRF24L01_WritePayload(packet, 9);
    }

    ++packet_counter;

//    radio.ce(HIGH);
//    delayMicroseconds(15);
    // It saves power to turn off radio after the transmission,
    // so as long as we have pins to do so, it is wise to turn
    // it back.
//    radio.ce(LOW);

    // Check and adjust transmission power. We do this after
    // transmission to not bother with timeout after power
    // settings change -  we have plenty of time until next
    // packet.
//    if (tx_power != Model.tx_power) {
        //Keep transmit power updated
//        tx_power = Model.tx_power;
//        NRF24L01_SetPower(tx_power);
//    }
}

void ProtocolYD717::init1()
{
    NRF24L01_Initialize();

    // CRC, radio on
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, BV(NRF24L01_00_EN_CRC) | BV(NRF24L01_00_PWR_UP)); 
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x3F);      // Auto Acknoledgement on all data pipes
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x3F);  // Enable all data pipes
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);   // 5-byte RX/TX address
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x1A); // 500uS retransmit t/o, 10 tries
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, RF_CHANNEL);      // Channel 3C
    NRF24L01_SetBitrate(NRF24L01_BR_1M);             // 1Mbps
    NRF24L01_SetPower(TXPOWER_100mW);
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
    NRF24L01_WriteReg(NRF24L01_0C_RX_ADDR_P2, 0xC3); // LSB byte of pipe 2 receive address
    NRF24L01_WriteReg(NRF24L01_0D_RX_ADDR_P3, 0xC4);
    NRF24L01_WriteReg(NRF24L01_0E_RX_ADDR_P4, 0xC5);
    NRF24L01_WriteReg(NRF24L01_0F_RX_ADDR_P5, 0xC6);
    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, PAYLOADSIZE);   // bytes of data payload for pipe 1
    NRF24L01_WriteReg(NRF24L01_12_RX_PW_P1, PAYLOADSIZE);
    NRF24L01_WriteReg(NRF24L01_13_RX_PW_P2, PAYLOADSIZE);
    NRF24L01_WriteReg(NRF24L01_14_RX_PW_P3, PAYLOADSIZE);
    NRF24L01_WriteReg(NRF24L01_15_RX_PW_P4, PAYLOADSIZE);
    NRF24L01_WriteReg(NRF24L01_16_RX_PW_P5, PAYLOADSIZE);
    NRF24L01_WriteReg(NRF24L01_17_FIFO_STATUS, 0x00); // Just in case, no real bits to write here
    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x3F);       // Enable dynamic payload length on all pipes

    // this sequence necessary for module from stock tx
    NRF24L01_ReadReg(NRF24L01_1D_FEATURE);
    NRF24L01_Activate(0x73);                          // Activate feature register
    NRF24L01_ReadReg(NRF24L01_1D_FEATURE);
    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x3F);       // Enable dynamic payload length on all pipes
    NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x07);     // Set feature bits on


    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, rx_tx_addr, 5);
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, rx_tx_addr, 5);

    // Check for Beken BK2421/BK2423 chip
    // It is done by using Beken specific activate code, 0x53
    // and checking that status register changed appropriately
    // There is no harm to run it on nRF24L01 because following
    // closing activate command changes state back even if it
    // does something on nRF24L01
    NRF24L01_Activate(0x53); // magic for BK2421 bank switch
    dbgprintf("Trying to switch banks\n");
    if (NRF24L01_ReadReg(NRF24L01_07_STATUS) & 0x80) {
        dbgprintf("BK2421 detected\n");
        // Beken registers don't have such nice names, so we just mention
        // them by their numbers
        // It's all magic, eavesdropped from real transfer and not even from the
        // data sheet - it has slightly different values
        NRF24L01_WriteRegisterMulti(0x00, (u8 *) "\x40\x4B\x01\xE2", 4);
        NRF24L01_WriteRegisterMulti(0x01, (u8 *) "\xC0\x4B\x00\x00", 4);
        NRF24L01_WriteRegisterMulti(0x02, (u8 *) "\xD0\xFC\x8C\x02", 4);
        NRF24L01_WriteRegisterMulti(0x03, (u8 *) "\x99\x00\x39\x21", 4);
        NRF24L01_WriteRegisterMulti(0x04, (u8 *) "\xD9\x96\x82\x1B", 4);
        NRF24L01_WriteRegisterMulti(0x05, (u8 *) "\x24\x06\x7F\xA6", 4);
        NRF24L01_WriteRegisterMulti(0x0C, (u8 *) "\x00\x12\x73\x00", 4);
        NRF24L01_WriteRegisterMulti(0x0D, (u8 *) "\x46\xB4\x80\x00", 4);
        NRF24L01_WriteRegisterMulti(0x04, (u8 *) "\xDF\x96\x82\x1B", 4);
        NRF24L01_WriteRegisterMulti(0x04, (u8 *) "\xD9\x96\x82\x1B", 4);
    } else {
        dbgprintf("nRF24L01 detected\n");
    }
    NRF24L01_Activate(0x53); // switch bank back

    // Implicit delay in callback
    // delay(50);
}


void ProtocolYD717::init2()
{
    // for bind packets set address to prearranged value known to receiver
    u8 bind_rx_tx_addr[] = {0x65, 0x65, 0x65, 0x65, 0x65};
//    if(Model.protocol == PROTOCOL_SymaX)
        for(u8 i=0; i < 5; i++) bind_rx_tx_addr[i]  = 0x60;

    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, bind_rx_tx_addr, 5);
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, bind_rx_tx_addr, 5);
}


void ProtocolYD717::init3()
{
    // set rx/tx address for data phase
    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, rx_tx_addr, 5);
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, rx_tx_addr, 5);
}


#ifdef YD717_TELEMETRY
void ProtocolYD717::updateTelemetry() {
  static u8 frameloss = 0;

  frameloss += NRF24L01_ReadReg(NRF24L01_08_OBSERVE_TX) >> 4;
  NRF24L01_WriteReg(NRF24L01_05_RF_CH, RF_CHANNEL);   // reset packet loss counter

  Telemetry.p.dsm.flog.frameloss = frameloss;
  TELEMETRY_SetUpdated(TELEM_DSM_FLOG_FRAMELOSS);
}
#endif


u16 ProtocolYD717::Callback()
{
    switch (phase) {
    case YD717_INIT1:
        sendPacket(0);      // receiver doesn't re-enter bind mode if connection lost...check if already bound
        phase = YD717_BIND3;
        break;

    case YD717_BIND2:
        if (counter == 0) {
            if (ackPacket() == PKT_PENDING)
                return PACKET_CHKTIME;             // packet send not yet complete
            init3();                         // change to data phase rx/tx address
            sendPacket(0);
            phase = YD717_BIND3;
        } else {
            if (ackPacket() == PKT_PENDING)
                return PACKET_CHKTIME;             // packet send not yet complete
            sendPacket(1);
            counter -= 1;
        }
        break;

    case YD717_BIND3:
        switch (ackPacket()) {
        case PKT_PENDING:
            return PACKET_CHKTIME;                 // packet send not yet complete
        case PKT_ACKED:
            phase = YD717_DATA;
//            PROTOCOL_SetBindState(0);
//            MUSIC_Play(MUSIC_DONE_BINDING);
            break;
        case PKT_TIMEOUT:
            init2();                         // change to bind rx/tx address
            counter = BIND_COUNT;
            phase = YD717_BIND2;
            sendPacket(1);
        }
        break;

    case YD717_DATA:
#ifdef YD717_TELEMETRY
        updateTelemetry();
#endif
        if (ackPacket() == PKT_PENDING)
            return PACKET_CHKTIME;                 // packet send not yet complete
        sendPacket(0);
        break;
    }
    return PACKET_PERIOD;                          // Packet every 8ms
}

void ProtocolYD717::TimerCallback()
{
    u16 time = Callback();
    getTimer().after((int)time, (void*)TimerCallback);
}

void ProtocolYD717::setRxTxAddr(u32 id)
{
    rx_tx_addr[0] = (id >> 24) & 0xFF;
    rx_tx_addr[1] = (id >> 16) & 0xFF;
    rx_tx_addr[2] = (id >>  8) & 0xFF;
    rx_tx_addr[3] = (id >>  0) & 0xFF;
    rx_tx_addr[4] = 0xC1; // always uses first data port
}

int ProtocolYD717::init(u32 id)
{
//    CLOCK_StopTimer();
    setRxTxAddr(id);
    packet_counter = 0;
    flags = 0;

    init1();
    phase = YD717_INIT1;

#ifdef YD717_TELEMETRY
    memset(&Telemetry, 0, sizeof(Telemetry));
    TELEMETRY_SetType(TELEM_DSM);
#endif

//    PROTOCOL_SetBindState(0xFFFFFFFF);
    getTimer().after(INITIAL_WAIT, TimerCallback);
    return 0;
}

int ProtocolYD717::deinit()
{
    CLOCK_StopTimer();
    return (NRF24L01_Reset() ? 1L : -1L);
}

int ProtocolYD717::reset()
{
    return deinit();
}

int ProtocolYD717::bind(u32 id)
{
    return init(id);
}

int ProtocolYD717::getChannels()
{
    return 6;
}

int ProtocolYD717::setPower(int power)
{
    NRF24L01_SetPower(power);
    return 0;
}
