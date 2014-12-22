
#include <SoftwareSerial.h>
#include <IRremote.h>
const int RECV_PIN   = 5;
const int BUTTON_PIN = 12;
const int STATUS_PIN = 13;

IRrecv  irrecv(RECV_PIN);
//IRsend  irsend;

int     nCodeType   = -1;   // The type of code
int     nToggle     = 0;    // The RC5/6 nToggle state
int     nCodeLen;           // The length of the code

unsigned long   lCode; // The code value if not raw
unsigned int    nRawCodes[RAWBUF]; // The durations if raw
decode_results  results;

//Servo   servoLR;
int     nPosLR = 0;
 
SoftwareSerial serialZigBee(2, 4); // RX, TX
 
void setup() 
{ 
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  serialZigBee.begin(115200);
	
 #if 0   
    servoLR.attach(9);
    irrecv.enableIRIn();
    pinMode(BUTTON_PIN, INPUT);
    pinMode(STATUS_PIN, OUTPUT);
#endif
} 

void loop() 
{ 
  while (Serial.available() > 0) {
    char inByte = Serial.read();
//	Serial.write(inByte);
    serialZigBee.write(inByte);
  }
  
  while (serialZigBee.available() > 0) {
    char inByte = serialZigBee.read();
    Serial.write(inByte);
  }  

/*
    int pos;
    
    for(pos = 0; pos < 180; pos++) {
        servoLR.write(pos);
        delay(15);
    }
    for(pos = 180; pos > 0; pos--) {
        servoLR.write(pos);
        delay(15);
    }
    
    //irrecv.enableIRIn();
    if (irrecv.decode(&results)) {
        storeCode(&results);
        irrecv.resume(); // resume receiver
    }
*/	
}

#if 0
void storeCode(decode_results *results) {
    nCodeType = results->decode_type;
    int count = results->rawlen;

    if (nCodeType == UNKNOWN) {
        Serial.println("Received unknown code, saving as raw");
        nCodeLen = results->rawlen - 1;
        // To store raw codes:
        // Drop first value (gap)
        // Convert from ticks to microseconds
        // Tweak marks shorter, and spaces longer to cancel out IR receiver distortion
        for (int i = 1; i <= nCodeLen; i++) {
            if (i % 2) {
                // Mark
                nRawCodes[i - 1] = results->rawbuf[i]*USECPERTICK - MARK_EXCESS;
                Serial.print(" m");
            } 
            else {
                // Space
                nRawCodes[i - 1] = results->rawbuf[i]*USECPERTICK + MARK_EXCESS;
                Serial.print(" s");
            }
            Serial.print(nRawCodes[i - 1], DEC);
        }
        Serial.println("");
    }
    else {
        if (nCodeType == NEC) {
            Serial.print("Received NEC: ");
            if (results->value == REPEAT) {
                // Don't record a NEC repeat value as that's useless.
                Serial.println("repeat; ignoring.");
                return;
            }
        } 
        else if (nCodeType == SONY) {
            Serial.print("Received SONY: ");
        } 
        else if (nCodeType == RC5) {
            Serial.print("Received RC5: ");
        } 
        else if (nCodeType == RC6) {
            Serial.print("Received RC6: ");
        } 
        else {
            Serial.print("Unexpected nCodeType ");
            Serial.print(nCodeType, DEC);
            Serial.println("");
        }
        Serial.println(results->value, HEX);
        lCode = results->value;
        nCodeLen   = results->bits;
    }
}

void sendCode(int repeat) {
    if (nCodeType == NEC) {
        if (repeat) {
            irsend.sendNEC(REPEAT, nCodeLen);
            Serial.println("Sent NEC repeat");
        } 
        else {
            irsend.sendNEC(lCode, nCodeLen);
            Serial.print("Sent NEC ");
            Serial.println(lCode, HEX);
        }
    } 
    else if (nCodeType == SONY) {
        irsend.sendSony(lCode, nCodeLen);
        Serial.print("Sent Sony ");
        Serial.println(lCode, HEX);
    } 
    else if (nCodeType == RC5 || nCodeType == RC6) {
        if (!repeat) {
            // Flip the nToggle bit for a new button press
            nToggle = 1 - nToggle;
        }
        // Put the nToggle bit into the code to send
        lCode = lCode & ~(1 << (nCodeLen - 1));
        lCode = lCode | (nToggle << (nCodeLen - 1));
        if (nCodeType == RC5) {
            Serial.print("Sent RC5 ");
            Serial.println(lCode, HEX);
            irsend.sendRC5(lCode, nCodeLen);
        } 
        else {
            irsend.sendRC6(lCode, nCodeLen);
            Serial.print("Sent RC6 ");
            Serial.println(lCode, HEX);
        }
    } 
    else if (nCodeType == UNKNOWN /* i.e. raw */) {
        // Assume 38 KHz
        irsend.sendRaw(nRawCodes, nCodeLen, 38);
        Serial.println("Sent raw");
    }
}
#endif
