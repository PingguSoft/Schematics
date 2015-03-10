#include <SPI.h>
#include "common.h"
#include "interface.h"
#include "Timer.h"
#include "ProtocolYD717.h"

int pin = 13;

ProtocolYD717 proto;

void setup() 
{
    Serial.begin(9600);
    while (!Serial); // wait for serial port to connect. Needed for Leonardo only

    pinMode(pin, OUTPUT);
    proto.getTimer().oscillate(pin, 300, LOW);
    //t.pulse(pin, 300, HIGH); 
    
    int tickEvent = proto.getTimer().every(2000, doSomething);
}

void loop()
{
    proto.update();
}

void doSomething()
{
    Serial.print("2 second tick: millis()=");
    Serial.println(millis());
}
