/* @file MultiKey.ino
|| @version 1.0
|| @author Mark Stanley
|| @contact mstanley@technologist.com
||
|| @description
|| | The latest version, 3.0, of the keypad library supports up to 10
|| | active keys all being pressed at the same time. This sketch is an
|| | example of how you can get multiple key presses from a keypad or
|| | keyboard.
|| #
*/

#include <Keypad.h>

uint8_t PIN_FORWARD = 11;
uint8_t PIN_BACKWARD = 12;

const byte ROWS = 1; //one row  (R1)
const byte COLS = 4; //four columns (L1,..L4)
char keys[ROWS][COLS] = {
{'1','2','3','4'}
};
byte rowPins[ROWS] = {6}; //connect to the row pinouts of the kpd
byte colPins[COLS] = {5, 4, 3, 2}; //connect to the column pinouts of the kpd

Keypad kpd = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

unsigned long loopCount;
unsigned long startTime;
String msg;


void setup() {
    Serial.begin(9600);
    loopCount = 0;
    startTime = millis();
    msg = "";
    pinMode (PIN_FORWARD, OUTPUT);
    digitalWrite(PIN_FORWARD, HIGH);
    pinMode (PIN_BACKWARD, OUTPUT);
    digitalWrite(PIN_BACKWARD, HIGH);
}


void loop() {
    loopCount++;
    if ( (millis()-startTime)>5000 ) {
        Serial.print("Average loops per second = ");
        Serial.println(loopCount/5);
        startTime = millis();
        loopCount = 0;
    }

    // Fills kpd.key[ ] array with up-to 10 active keys.
    // Returns true if there are ANY active keys.
    if (kpd.getKeys())
    {
        for (int i=0; i<LIST_MAX; i++)   // Scan the whole key list.
        {
            if ( kpd.key[i].stateChanged )   // Only find keys that have changed state.
            {
                switch (kpd.key[i].kstate) {  // Report active key state : IDLE, PRESSED, HOLD, or RELEASED
                    case PRESSED:
                    msg = " PRESSED.";
                    switch (kpd.key[i].kchar) {
                        case '2':
                            digitalWrite(PIN_FORWARD, LOW);
                        break;
                        case '4':
                            digitalWrite(PIN_BACKWARD, LOW);
                        break;
                    }
                    break;
                    case HOLD:
                    msg = " HOLD.";
                    break;
                    case RELEASED:
                    msg = " RELEASED.";
                    switch (kpd.key[i].kchar) {
                        case '2':
                            digitalWrite(PIN_FORWARD, HIGH);
                        break;
                        case '4':
                            digitalWrite(PIN_BACKWARD, HIGH);
                        break;
                    }
                    break;
                    case IDLE:
                    msg = " IDLE.";
                }
                Serial.println(i);
                Serial.print("Key ");
                Serial.print(kpd.key[i].kchar);
                Serial.println(msg);
                
            }
        }
    }
}  // End loop