#include <SPI.h>
#include "common.h"
#include "interface.h"
#include "Timer.h"
#include "ProtocolSyma.h"

ProtocolSyma proto;
int incomingByte = 0;

void setup()
{
    Serial.begin(57600);
    while (!Serial); // wait for serial port to connect. Needed for Leonardo only

    Serial.println("START!!!");

    proto.init(0xb2c54a2ful);
}

extern void tmr2();
extern void tmr3();
extern void tmrStop();

void loop()
{
    proto.update();
    if (Serial.available() > 0) {
		// read the incoming byte:
        incomingByte = Serial.read();
		if (incomingByte == 'a') {
			tmrStop();
			tmr2();
		}
		else if (incomingByte == 'b') {
			tmrStop();
			tmr3();
		}
	}
}

