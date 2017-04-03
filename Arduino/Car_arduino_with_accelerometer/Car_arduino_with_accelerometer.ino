//*************************** Includes begin *********************************

#include <SPI.h>
#include <VirtualWire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

//*************************** Includes end *********************************


//*************************** Settings begin *********************************

byte drive_PWM = 36;
byte turn_PWM = 70;

//*************************** Settings end *********************************


//*************************** Constants begin *********************************

// motors driver (Monster shield) pins:
#define INA1 7
#define INB1 8
#define INA2 4
#define INB2 9
#define PWM1 5
#define PWM2 6
#define EN1 A0
#define EN2 A1
#define CS1 A2
#define CS2 A3

// LCD pins:
#define OLED_CLK   25
#define OLED_MOSI  24
#define OLED_CS    26
#define OLED_DC    28
#define OLED_RESET 30

// receiver pins:
#define rx_pin 23

// Arduino --> MPU6050 pins:
    // Arduino Nano:
    // A4 --> SDA
    // A5 --> SCL
    // D2 --> INT (interrupt, if needed)
    
    // Arduino Mega:
    // A20 --> SDA
    // A21 --> SCL
    // D2 --> INT (interrupt, if needed)


//*************************** Constants end *********************************


//*************************** Variables begin *********************************


Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

uint8_t buf[VW_MAX_MESSAGE_LEN];
uint8_t buflen = VW_MAX_MESSAGE_LEN;
String buf_string;
int command_hesh; 
int current_comand;
bool training_mode = false;


// **** Begin v_measurement vars ****

  unsigned long t_current, t_old, Vtimer, t_start, t_measurement = 100;
  float Vy_current =0, v_start, v_target = 10;
  float accY_old;
  bool start_measurement = false;
// **** End v_measurement vars ****

// **** Begin acceleration vars ****
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// **** End acceleration vars ****

//*************************** Variables end *********************************


//*********************** Functions begin ********************************

void stop_car()
{
  digitalWrite(INA1, LOW);
  digitalWrite(INB1, LOW);
  digitalWrite(INA2, LOW);
  digitalWrite(INB2, LOW);
  analogWrite(PWM1, 0);
  analogWrite(PWM2, 0);  
}

void afterburner ()
{
  digitalWrite(INA1, HIGH);
  digitalWrite(INB1, LOW);
  digitalWrite(INA2, HIGH);
  digitalWrite(INB2, LOW);
  analogWrite(PWM1, (drive_PWM + 15));
  analogWrite(PWM2, (drive_PWM + 15));
}

void forward ()
{
  digitalWrite(INA1, LOW);
  digitalWrite(INB1, HIGH);
  digitalWrite(INA2, LOW);
  digitalWrite(INB2, HIGH);
  analogWrite(PWM1, drive_PWM);
  analogWrite(PWM2, drive_PWM);
}

void backward ()
{
  digitalWrite(INA1, HIGH);
  digitalWrite(INB1, LOW);
  digitalWrite(INA2, HIGH);
  digitalWrite(INB2, LOW);
  analogWrite(PWM1, drive_PWM);
  analogWrite(PWM2, drive_PWM);
}

void turn_right ()
{
  digitalWrite(INA1, LOW);
  digitalWrite(INB1, HIGH);
  digitalWrite(INA2, HIGH);
  digitalWrite(INB2, LOW);
  analogWrite(PWM1, turn_PWM);
  analogWrite(PWM2, turn_PWM);
}

void turn_left ()
{
  digitalWrite(INA1, HIGH);
  digitalWrite(INB1, LOW);
  digitalWrite(INA2, LOW);
  digitalWrite(INB2, HIGH);
  analogWrite(PWM1, turn_PWM);
  analogWrite(PWM2, turn_PWM);
}

String GetMessage()
{
    int i;
    buf_string = "";
    command_hesh = 0; 
   
    for (i = 0; i < buflen; i++)
    {
      command_hesh += buf[i];
      buf_string += char(buf[i]); 
    }
}

//*********************** Functions end ********************************


//*********************** Setup begin **********************************

void setup()   
{      
            
    // Initialise the IO and ISR
    //  vw_set_ptt_inverted(true); // Required for DR3100
    vw_setup(2000); // Bits per sec

    vw_rx_start(); // Start the receiver PLL running
    pinMode(rx_pin,INPUT);
    vw_set_rx_pin(rx_pin);

    //************** Begin display setup ************
    // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
    display.begin(SSD1306_SWITCHCAPVCC);
    // **** init done ****
    
    // Clear display the buffer.

    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.display();
    //************** End display setup ************
    

    // setup motor driver pins
    pinMode (INA1, OUTPUT);
    pinMode (INB1, OUTPUT);
    pinMode (INA2, OUTPUT);
    pinMode (INB2, OUTPUT);
    pinMode (PWM1, OUTPUT);
    pinMode (PWM2, OUTPUT);
    //  pinMode (EN1, OUTPUT);
    //  pinMode (EN2, OUTPUT);
    //  pinMode (CS1, INPUT);
    //  pinMode (CS21, INPUT);

    stop_car();

    // Begin change PWM from 500 Hz to 4000 Hz 
    //(http://forum.arduino.cc/index.php?topic=72092.0)
    int myEraser = 7;      // this is 111 in binary and is used as an eraser
    TCCR3B &= ~myEraser;   // this operation (AND plus NOT),  set the three bits in TCCR2B to 0
    TCCR4B &= ~myEraser;   // this operation (AND plus NOT),  set the three bits in TCCR2B to 0

    // now that CS02, CS01, CS00  are clear, we write on them a new value:
    int myPrescaler = 2;         // this could be a number in [1 , 6]. In this case, 3 corresponds in binary to 011.   
    TCCR3B |= myPrescaler;  // this operation (OR), replaces the last three bits in TCCR2B with our new value 011  
    TCCR4B |= myPrescaler;  // this operation (OR), replaces the last three bits in TCCR2B with our new value 011  
    // End change PWM from 500 Hz to 4000 Hz

    Serial.begin(115200);
    
    //********** Begin setup accelerometer *********
    
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize device MPU6050
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    
    // set offsets of this particular MPU6050
    mpu.setXGyroOffset(25);
    mpu.setYGyroOffset(45);
    mpu.setZGyroOffset(44);
    mpu.setXAccelOffset(-5858);
    mpu.setYAccelOffset(1349);
    mpu.setZAccelOffset(1190);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    //********** End setup accelerometer *********
    
    //********** Begin setup velocity measurement*********
    accY_old = 0;
    t_old = millis();
    Vtimer = 40000;  
    //********** End setup velocity measurement*********    
}

//*********************** Setup end **********************************


//*********************** Loop begin **********************************

void loop() 
{
    buf[VW_MAX_MESSAGE_LEN];
    buflen = VW_MAX_MESSAGE_LEN;

    if (vw_get_message(buf, &buflen)) // Non-blocking
    {
        digitalWrite(12, true); // Flash a light to show received good message
        // Message with a good checksum received, dump it.

        GetMessage();

        switch (command_hesh)
        {
          case (5):      // 5
            stop_car();
          break;
          
          case (9):          //  +
            drive_PWM = drive_PWM + 1;
          break;  

          case (7):           // -
            drive_PWM = drive_PWM - 1;
          break;     

          case (3):          // q
            turn_PWM += 1;
          break;  
          
          case (1):            // a
            turn_PWM -= 1;
          break;  
          
          case (0):            // t
            training_mode = true;
          break;      
          
          default:
            if ((command_hesh == current_comand)||(training_mode))
            {
                switch (command_hesh)
                {
                  case (47):      // /
                    afterburner();
                    delay(20);
                  break;

                  case (8):         // 8
                    forward();
                  break;

                  case (2):         // 2
                    backward();
                  break;

                  case (4):         // 4 
                    turn_left();
                  break;

                  case (6):       //  6
                    turn_right();
                  break;      
                }
            }
            else
            {
                current_comand = command_hesh;      
                stop_car();
            //    delay(200);
                t_start = millis();
                v_start = Vy_current;
                start_measurement = true;
            }
        }    

        display.setCursor(0,0);
        display.clearDisplay();

        display.println(micros());

        display.println(buf_string);
        display.println(command_hesh);
        display.println(drive_PWM);
        display.println(turn_PWM);
            
        display.display();
        digitalWrite(12, false);
        
        Serial.println(buf_string);
    }
  
    
    
    //*********************** Begin get acceleration and velocity loop **************
    // get INT_STATUS byte
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } 
    else if (mpuIntStatus & 0x02) 
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;    
        

        // get real acceleration, adjusted to remove gravity
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    /*  Serial.print("areal\t");
        Serial.print(aaReal.x);
        Serial.print("\t");
        Serial.println(aaReal.y);
        Serial.print("\t");
        Serial.println(aaReal.z);
    */
        t_current = millis();
        Vy_current += (t_current-t_old)*(aaReal.y+accY_old)/2000;
        t_old = t_current;
        accY_old = aaReal.y;
        if (t_old > Vtimer) 
        {
            Vtimer = Vtimer+40000;
            Vy_current =0;
        }
        //  Serial.print(" Vy = ");
        //  Serial.println(Vx_new);
    }
    //*********************** End get acceleration and velocity loop **************   
    
    //*********************** Begin v_measurement loop ************
    if ((start_measurement == true)&&((millis()-t_start) > t_measurement))
    {
        // look v_current
        if (Vy_current-v_start > v_target)
        {
            Serial.println("Ok");
        //    Serial.println(Vy_current);
            Serial.println(Vy_current-v_start);
            Serial.println(millis()-t_start);
         //   delay(2000);
        }
        else
        {
            Serial.println("Not Ok");
         //   Serial.println(Vy_current);
            Serial.println(Vy_current-v_start);
            Serial.println(millis()-t_start);
          //  delay(2000);
        }        
        start_measurement = false;
    }        
    //**** End v_measurement loop
        
        
}  

//*********************** Loop end **********************************



