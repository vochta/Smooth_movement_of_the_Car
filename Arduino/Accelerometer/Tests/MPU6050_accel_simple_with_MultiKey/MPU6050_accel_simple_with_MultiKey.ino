/*
    MPU6050 Triple Axis Gyroscope & Accelerometer. Simple Accelerometer Example.
    Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-zyroskop-i-akcelerometr-mpu6050.html
    GIT: https://github.com/jarzebski/Arduino-MPU6050
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski
*/

#include <Wire.h>
#include <MPU6050_Jarzebski.h>
#include <Keypad.h>

MPU6050 mpu;

// **** Begin Keypad vars ****



uint8_t PIN_FORWARD = 11;
uint8_t PIN_BACKWARD = 12;
const byte ROWS = 1; //one row	(R1)
const byte COLS = 4; //four columns (L1,..L4)
char keys[ROWS][COLS] = {
{'1','2','3','4'}
};
byte rowPins[ROWS] = {10}; //connect to the row pinouts of the kpd
byte colPins[COLS] = {9,8,7,6}; //connect to the column pinouts of the kpd

Keypad kpd = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

unsigned long loopCount;
unsigned long startTime;
String msg;

// **** End Keypad vars ****




void setup() 
{

    loopCount = 0;
    startTime = millis();
    msg = "";
    pinMode (PIN_FORWARD, OUTPUT);
    digitalWrite(PIN_FORWARD, HIGH);
    pinMode (PIN_BACKWARD, OUTPUT);
    digitalWrite(PIN_BACKWARD, HIGH);


  Serial.begin(115200);

  Serial.println("Initialize MPU6050");

  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  // If you want, you can set accelerometer offsets
  // mpu.setAccelOffsetX();
  // mpu.setAccelOffsetY();
  // mpu.setAccelOffsetZ();
  
  checkSettings();
}

void checkSettings()
{
  Serial.println();
  
  Serial.print(" * Sleep Mode:            ");
  Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");
  
  Serial.print(" * Clock Source:          ");
  switch(mpu.getClockSource())
  {
    case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
    case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println("Internal 8MHz oscillator"); break;
  }
  
  Serial.print(" * Accelerometer:         ");
  switch(mpu.getRange())
  {
    case MPU6050_RANGE_16G:            Serial.println("+/- 16 g"); break;
    case MPU6050_RANGE_8G:             Serial.println("+/- 8 g"); break;
    case MPU6050_RANGE_4G:             Serial.println("+/- 4 g"); break;
    case MPU6050_RANGE_2G:             Serial.println("+/- 2 g"); break;
  }  

  Serial.print(" * Accelerometer offsets: ");
  Serial.print(mpu.getAccelOffsetX());
  Serial.print(" / ");
  Serial.print(mpu.getAccelOffsetY());
  Serial.print(" / ");
  Serial.println(mpu.getAccelOffsetZ());
  
  Serial.println();
}

void loop()
{
//**** Begin Keypad loop ******
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
//**** End Keypad loop





//  Vector rawAccel = mpu.readRawAccel();
  Vector normAccel = mpu.readNormalizeAccel();

/* Serial.print(" Xraw = ");
  Serial.print(rawAccel.XAxis);
  Serial.print(" Yraw = ");
  Serial.print(rawAccel.YAxis);
  Serial.print(" Zraw = ");

  Serial.println(rawAccel.ZAxis);
  */
  Serial.print(" Xnorm = ");
  Serial.print(normAccel.XAxis);
  Serial.print(" Ynorm = ");
  Serial.print(normAccel.YAxis);
  Serial.print(" Znorm = ");
  Serial.println(normAccel.ZAxis);
  
  delay(100);
}


