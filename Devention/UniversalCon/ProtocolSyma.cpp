#include <SPI.h>
#include "common.h"
#include "interface.h"
#include "ProtocolSyma.h"
#include "utils.h"

#define PAYLOADSIZE         10      // receive data pipes set to this size, but unused
#define MAX_PACKET_SIZE     16      // X11,X12,X5C-1 10-byte, X5C 16-byte

#define BIND_COUNT         345
#define PACKET_PERIOD        4 //000
#define INITIAL_WAIT         1 // 500
#define FIRST_PACKET_DELAY  12 //000

#define FLAG_FLIP         0x01
#define FLAG_VIDEO        0x02
#define FLAG_PICTURE      0x04

#define MAX_RF_CHANNELS     17

enum {
    SYMAX_INIT1 = 0,
    SYMAX_BIND2,
    SYMAX_BIND3,
    SYMAX_DATA
};

enum {
    PROTOOPTS_OTHERS  = 0,
    PROTOOPTS_X5C_X2  = 1,
    LAST_PROTO_OPT,
};

static u8     current_chan;
static u8     chans[MAX_RF_CHANNELS];
static u8     num_rf_channels;

static u8     proto = PROTOOPTS_OTHERS;
static u8     packet[MAX_PACKET_SIZE];
static u8     packet_size;
static u16    counter;
static u32    packet_counter;
static u8     tx_power, flags;
static u8     rx_tx_addr[5];
static u8     phase;
static Timer  mTimer;

#include <stdarg.h>

void p(char *fmt, ... )
{
  char buf[128]; // resulting string limited to 128 chars
  va_list args;
  va_start (args, fmt );
  vsnprintf(buf, 128, fmt, args);
  va_end (args);
  Serial.print(buf);
}

void p(const __FlashStringHelper *fmt, ... ){
  char buf[128]; // resulting string limited to 128 chars
  va_list args;
  va_start (args, fmt);
#ifdef __AVR__
  vsnprintf_P(buf, sizeof(buf), (const char *)fmt, args); // progmem for AVR
#else
  vsnprintf(buf, sizeof(buf), (const char *)fmt, args); // for the rest of the world
#endif
  va_end(args);
  Serial.print(buf);
}


static u8 checksum(u8 *data)
{
    u8 sum = data[0];

    for (int i=1; i < packet_size-1; i++)
        if (proto == PROTOOPTS_X5C_X2)
            sum += data[i];
        else
            sum ^= data[i];

    return sum + ((proto == PROTOOPTS_X5C_X2) ? 0 : 0x55);
}

static u8 checkStatus()
{
    u8 stat = NRF24L01_ReadReg(NRF24L01_07_STATUS);
    p(F("checkStatus :%x\n"), stat);

    switch (stat & (BV(NRF24L01_07_TX_DS) | BV(NRF24L01_07_MAX_RT))) {
    case BV(NRF24L01_07_TX_DS):
        return PKT_ACKED;
    case BV(NRF24L01_07_MAX_RT):
        NRF24L01_WriteReg(NRF24L01_07_STATUS, BV(NRF24L01_07_MAX_RT));
        return PKT_TIMEOUT;
    }
    return PKT_PENDING;
}


#define BABS(X) (((X) < 0) ? -(u8)(X) : (X))
static u8 getChannel(CH_T id)
{
    s32 ch = Protocol::getControl(id);
    if (ch < CHAN_MIN_VALUE) {
        ch = CHAN_MIN_VALUE;
    } else if (ch > CHAN_MAX_VALUE) {
        ch = CHAN_MAX_VALUE;
    }

    u8 ret = (u8) ((ch < 0 ? 0x80 : 0) | BABS(ch * 127 / CHAN_MAX_VALUE));


//    if (id == CH_THROTTLE)
//      p(F("THR:%ld => %02x\n"), ch, ret);

    return ret;
}

#define X5C_CHAN2TRIM(X) ((((X) & 0x80 ? 0xff - (X) : 0x80 + (X)) >> 2) + 0x20)

static void build_packet_x5c(u8 bind)
{
    if (bind) {
        memset(packet, 0, packet_size);
        packet[7] = 0xae;
        packet[8] = 0xa9;
        packet[14] = 0xc0;
        packet[15] = 0x17;
    } else {
        packet[0] = getChannel(CH_THROTTLE);
        packet[0] = (packet[0] & 0x80) ? (0xff - packet[0]) : (0x80 + packet[0]);
        packet[1] = getChannel(CH_RUDDER);
        packet[2] = getChannel(CH_ELEVATOR) ^ 0x80;  // reversed from default
        packet[3] = getChannel(CH_AILERON);
        packet[4] = X5C_CHAN2TRIM(getChannel(CH_RUDDER) ^ 0x80);     // drive trims for extra control range
        packet[5] = X5C_CHAN2TRIM(getChannel(CH_ELEVATOR));
        packet[6] = X5C_CHAN2TRIM(getChannel(CH_AILERON) ^ 0x80);
        packet[7] = 0xae;
        packet[8] = 0xa9;
        packet[9] = 0x00;
        packet[10] = 0x00;
        packet[11] = 0x00;
        packet[12] = 0x00;
        packet[13] = 0x00;
        packet[14] = (flags & FLAG_VIDEO   ? 0x10 : 0x00)
                   | (flags & FLAG_PICTURE ? 0x08 : 0x00)
                   | (flags & FLAG_FLIP    ? 0x01 : 0x00)
                   | 0x04;  // always high rates (bit 3 is rate control)
        packet[15] = checksum(packet);
    }
}

static void build_packet(u8 bind)
{
    if (bind) {
        packet[0] = rx_tx_addr[4];
        packet[1] = rx_tx_addr[3];
        packet[2] = rx_tx_addr[2];
        packet[3] = rx_tx_addr[1];
        packet[4] = rx_tx_addr[0];
        packet[5] = 0xaa;
        packet[6] = 0xaa;
        packet[7] = 0xaa;
        packet[8] = 0x00;
    } else {
        packet[0] = getChannel(CH_THROTTLE);
        packet[0] = (packet[0] & 0x80) ? (0xff - packet[0]) : (0x80 + packet[0]);
        packet[1] = getChannel(CH_ELEVATOR);
        packet[2] = getChannel(CH_RUDDER);
        packet[3] = getChannel(CH_AILERON);
        packet[4] = (flags & FLAG_VIDEO   ? 0x80 : 0x00)
                  | (flags & FLAG_PICTURE ? 0x40 : 0x00);
        // use trims to extend controls
        packet[5] = (getChannel(CH_ELEVATOR) >> 2) | 0xc0; // always high rates (bit 7 is rate control)
        packet[6] = (getChannel(CH_RUDDER) >> 2)   | (flags & FLAG_FLIP  ? 0x40 : 0x00);
        packet[7] = getChannel(CH_AILERON) >> 2;
        packet[8] = 0x00;
    }
    packet[9] = checksum(packet);

}

int frameloss = 0;
static void sendPacket(u8 bind)
{
    if (proto == PROTOOPTS_X5C_X2)
      build_packet_x5c(bind);
    else
      build_packet(bind);

    // clear packet status bits and TX FIFO
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x2e);
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, chans[current_chan]);
    NRF24L01_FlushTx();

    NRF24L01_WritePayload(packet, packet_size);

#if 0
    p(F("send packet : %d\n"), bind);
    for (u8 i = 0; i < packet_size; i++) {
      p(F("%02x, "), packet[i]);
    }
    p(F("\n"));
#endif

    if (packet_counter++ % 2) {   // use each channel twice
        current_chan = (current_chan + 1) % num_rf_channels;
    }

//    p(F("SEND PACKET bind:%d :%d\n"), bind, packet_counter);

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

//    p(F("SEND PACKET 2 : %x\n"), NRF24L01_ReadReg(NRF24L01_17_FIFO_STATUS));

    u8 stat;

    do {
      stat = NRF24L01_ReadReg(NRF24L01_07_STATUS);
//      p(F("SEND PACKET bind:%d : %x\n"), bind, stat);
    } while ((stat & BV(5)) == 0);
    NRF24L01_WriteReg(NRF24L01_07_STATUS, BV(5));

//    frameloss += NRF24L01_ReadReg(NRF24L01_08_OBSERVE_TX) >> 4;
//    p(F("SEND PACKET loss:%d\n"), frameloss);

}


static u32 initRxTxAddr()
{
    u32 lfsr = 0xb2c54a2ful;

    // Pump zero bytes for LFSR to diverge more
    for (u8 i = 0; i < sizeof(lfsr); ++i)
      rand32_r(&lfsr, 0);

    rx_tx_addr[4] = 0xa2;
    for (u8 i = 0; i < sizeof(rx_tx_addr)-1; ++i) {
        rx_tx_addr[i] = lfsr & 0xff;
        rand32_r(&lfsr, i);
    }

    p(F("ID:%08lx\n"), lfsr);

    return lfsr;
}

static void symax_init()
{
    const u8 bind_rx_tx_addr[] = {0xab,0xac,0xad,0xae,0xaf};
    const u8 rx_tx_addr_x5c[] = {0x6d,0x6a,0x73,0x73,0x73};   // X5C uses same address for bind and data

    NRF24L01_Initialize();

    NRF24L01_SetTxRxMode(TX_EN);

    NRF24L01_ReadReg(NRF24L01_07_STATUS);
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, BV(NRF24L01_00_EN_CRC) | BV(NRF24L01_00_CRCO));
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknoledgement
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x3F);  // Enable all data pipes (even though not used?)
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);   // 5-byte RX/TX address
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0xff); // 4mS retransmit t/o, 15 tries (retries w/o AA?)
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, 0x08);

    if (proto == PROTOOPTS_X5C_X2) {
      NRF24L01_SetBitrate(NRF24L01_BR_1M);
      packet_size = 16;
    } else {
      NRF24L01_SetBitrate(NRF24L01_BR_250K);
      packet_size = 10;
    }

    NRF24L01_SetPower(TXPOWER_100mW);
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
    NRF24L01_WriteReg(NRF24L01_08_OBSERVE_TX, 0x00);
    NRF24L01_WriteReg(NRF24L01_09_CD, 0x00);
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

    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR,
                                (proto == PROTOOPTS_X5C_X2) ? rx_tx_addr_x5c : bind_rx_tx_addr,
                                5);

    NRF24L01_ReadReg(NRF24L01_07_STATUS);

#if 0
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
        NRF24L01_WriteRegisterMulti(0x04, (u8 *) "\xF9\x96\x82\x1B", 4);
        NRF24L01_WriteRegisterMulti(0x05, (u8 *) "\x24\x06\x7F\xA6", 4);
        NRF24L01_WriteRegisterMulti(0x06, (u8 *) "\x00\x00\x00\x00", 4);
        NRF24L01_WriteRegisterMulti(0x07, (u8 *) "\x00\x00\x00\x00", 4);
        NRF24L01_WriteRegisterMulti(0x08, (u8 *) "\x00\x00\x00\x00", 4);
        NRF24L01_WriteRegisterMulti(0x09, (u8 *) "\x00\x00\x00\x00", 4);
        NRF24L01_WriteRegisterMulti(0x0A, (u8 *) "\x00\x00\x00\x00", 4);
        NRF24L01_WriteRegisterMulti(0x0B, (u8 *) "\x00\x00\x00\x00", 4);
        NRF24L01_WriteRegisterMulti(0x0C, (u8 *) "\x00\x12\x73\x00", 4);
        NRF24L01_WriteRegisterMulti(0x0D, (u8 *) "\x46\xB4\x80\x00", 4);
        NRF24L01_WriteRegisterMulti(0x0E, (u8 *) "\x41\x10\x04\x82\x20\x08\x08\xF2\x7D\xEF\xFF", 11);
        NRF24L01_WriteRegisterMulti(0x04, (u8 *) "\xFF\x96\x82\x1B", 4);
        NRF24L01_WriteRegisterMulti(0x04, (u8 *) "\xF9\x96\x82\x1B", 4);
    } else {
        dbgprintf("nRF24L01 detected\n");
    }
    NRF24L01_Activate(0x53); // switch bank back
#endif

    NRF24L01_FlushTx();
    NRF24L01_ReadReg(NRF24L01_07_STATUS);
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x0e);
    NRF24L01_ReadReg(NRF24L01_00_CONFIG);
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x0c);
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x0e);  // power on

    p(F("symax_init : %ld\n"), millis());
}



// write a strange first packet to RF channel 8 ...
const PROGMEM u8 first_packet[] = {0xf9, 0x96, 0x82, 0x1b, 0x20, 0x08, 0x08, 0xf2, 0x7d, 0xef, 0xff, 0x00, 0x00, 0x00, 0x00};
const PROGMEM u8 chans_bind[] = {0x4b, 0x30, 0x40, 0x2e};
const PROGMEM u8 chans_bind_x5c[] = {0x27, 0x1b, 0x39, 0x28, 0x24, 0x22, 0x2e, 0x36,
                       0x19, 0x21, 0x29, 0x14, 0x1e, 0x12, 0x2d, 0x18};

static void symax_init1()
{
    NRF24L01_FlushTx();
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, 0x08);
    memcpy_P(chans, first_packet, sizeof(first_packet));
    NRF24L01_WritePayload(chans, sizeof(first_packet));

    if (proto == PROTOOPTS_X5C_X2) {
      num_rf_channels = sizeof(chans_bind_x5c);
      memcpy_P(chans, chans_bind_x5c, num_rf_channels);
    } else {
      initRxTxAddr();   // make info available for bind packets
      num_rf_channels = sizeof(chans_bind);
      memcpy_P(chans, chans_bind, num_rf_channels);
    }

//    for (u8 i = 0; i < num_rf_channels; i++)
//          p(F("symax_init1 CH %d => %02x\n"), i, chans[i]);

    current_chan = 0;
    packet_counter = 0;

    p(F("symax_init1 : %ld\n"), millis());
}


const PROGMEM u8 start_chans_1[] = {0x0a, 0x1a, 0x2a, 0x3a};
const PROGMEM u8 start_chans_2[] = {0x2a, 0x0a, 0x42, 0x22};
const PROGMEM u8 start_chans_3[] = {0x1a, 0x3a, 0x12, 0x32};

// channels determined by last byte of tx address
static void set_channels(u8 address) {
  u8 laddress = address & 0x1f;
  u8 i;
  u32 *pchans = (u32 *)chans;   // avoid compiler warning

  num_rf_channels = 4;

  if (laddress < 0x10) {
    if (laddress == 6) laddress = 7;
    for(i=0; i < num_rf_channels; i++) {
      chans[i] = pgm_read_byte(start_chans_1 + i) + laddress;
    }
  } else if (laddress < 0x18) {
    for(i=0; i < num_rf_channels; i++) {
      chans[i] = pgm_read_byte(start_chans_2 + i) + (laddress & 0x07);
    }
    if (laddress == 0x16) {
      chans[0] += 1;
      chans[1] += 1;
    }
  } else if (laddress < 0x1e) {
    for(i=0; i < num_rf_channels; i++) {
      chans[i] = pgm_read_byte(start_chans_3 + i) + (laddress & 0x07);
    }
  } else if (laddress == 0x1e) {
      *pchans = 0x38184121;
  } else {
      *pchans = 0x39194121;
  }
}

const PROGMEM u8 chans_data_x5c[] = {0x1d, 0x2f, 0x26, 0x3d, 0x15, 0x2b, 0x25, 0x24,
                                     0x27, 0x2c, 0x1c, 0x3e, 0x39, 0x2d, 0x22};
static void symax_init2()
{
    if (proto == PROTOOPTS_X5C_X2) {
      num_rf_channels = sizeof(chans_data_x5c);
      memcpy_P(chans, chans_data_x5c, num_rf_channels);
    } else {
      set_channels(rx_tx_addr[0]);
      NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, rx_tx_addr, 5);
    }
    current_chan = 0;
    packet_counter = 0;
    p(F("symax_init2 : %ld\n"), millis());
}

void tmr2(void);
void tmr3(void);

static u16 callState()
{
    switch (phase) {
    case SYMAX_INIT1:
        symax_init1();
        phase = SYMAX_BIND2;
        return FIRST_PACKET_DELAY;
        break;

    case SYMAX_BIND2:
        counter = BIND_COUNT;
        phase   = SYMAX_BIND3;
        sendPacket(1);
        break;

    case SYMAX_BIND3:
        if (counter == 0) {
            symax_init2();
            phase = SYMAX_DATA;
            p(F("Bind Done : %ld\n"), millis());
        } else {
            sendPacket(1);
            counter--;
        }
        break;

    case SYMAX_DATA:
        sendPacket(0);
        break;
    }
    return PACKET_PERIOD;
}

static void tmrCallback(void)
{
    u16 time = callState();
    mTimer.after(time, tmrCallback);
}

int8_t hTmr2 = -1;
s32 thr = CHAN_MIN_VALUE;

void tmr2(void)
{
    if (thr < CHAN_MAX_VALUE)
      thr += 5;
    else {
      p(F("tmr2 !!!\n"));
      return;
    }

    Protocol::injectControl(CH_THROTTLE, thr);
    hTmr2 = mTimer.after(1, tmr2);
}

void tmr3(void)
{
    if (thr > CHAN_MIN_VALUE)
      thr -= 5;
    else {
      p(F("tmr3 !!!\n"));
      return;
    }

    Protocol::injectControl(CH_THROTTLE, thr);
    hTmr2 = mTimer.after(1, tmr3);
}

void tmrStop()
{
  if (hTmr2 > 0)
    mTimer.stop(hTmr2);
}

Timer ProtocolSyma::getTimer()
{
    return mTimer;
}

void ProtocolSyma::update()
{
    mTimer.update();
}

int ProtocolSyma::init(u32 id)
{
    packet_counter = 0;
    flags = 0;

    symax_init();
    phase = SYMAX_INIT1;

    for (u8 i = 0; i <= 0x1D; i++) {
      p(F("0x%02x => 0x%02x\n"), i, NRF24L01_ReadReg(i));
    }

//    PROTOCOL_SetBindState(0xFFFFFFFF);
    mTimer.after(INITIAL_WAIT, tmrCallback);
    p(F("init : %ld\n"), millis());

    return 0;
}

int ProtocolSyma::deinit()
{
//    CLOCK_StopTimer();
    NRF24L01_Initialize();
    return (NRF24L01_Reset() ? 1L : -1L);
}

int ProtocolSyma::reset()
{
    return deinit();
}

int ProtocolSyma::bind(u32 id)
{
    return init(id);
}

int ProtocolSyma::getChannels()
{
    return 6;
}

int ProtocolSyma::setPower(int power)
{
    NRF24L01_SetPower(power);
    return 0;
}
