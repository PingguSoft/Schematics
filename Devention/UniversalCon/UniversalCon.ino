#include <stdio.h>
#include <Arduino.h>
#include <avr/pgmspace.h>
#include <SPI.h>

#include "common.h"
#include "utils.h"
#include "ProtocolSyma.h"
#include "ProtocolYD717.h"

Protocol *proto = new ProtocolSyma();
int incomingByte = 0;

void setup()
{
    Serial.begin(57600);
    while (!Serial); // wait for serial port to connect. Needed for Leonardo only

    printf(F("START!!!\n"));
    printf(F("freeRAM : %d\n"), freeRam());

    proto->setDevID(0xb2c54a2ful);
    proto->init();
}

int freeRam() {
    extern int __heap_start, *__brkval; 
    int v; 
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

void loop()
{
    if (proto)
        proto->loop();

    if (Serial.available() > 0) {
        // read the incoming byte:
        incomingByte = Serial.read();
        switch (incomingByte) {
        case 'a':
            proto->test(1);
            break;

        case 'b':
            proto->test(2);
            break;

        case 'c':
            printf(F("freeRAM : %d\n"), freeRam());
            break;

        case 'z':
            if (proto)
                delete proto;
            proto = new ProtocolSyma();
            proto->init();
            break;

        case 'x':
            if (proto)
                delete proto;
            proto = new ProtocolYD717();
            proto->init();
            break;

        case 'q':
            if (proto) {
                delete proto;
                proto = NULL;
            }
            break;
        }
    }
}

