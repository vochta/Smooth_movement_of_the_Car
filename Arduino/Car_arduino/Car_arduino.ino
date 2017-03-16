//first edit at 25.08.16:
//  - PWM changed 

//*************************** Includes begin *********************************

#include <SPI.h>
#include <VirtualWire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

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
#define OLED_CLK   22
#define OLED_MOSI  24
#define OLED_CS    26
#define OLED_DC    28
#define OLED_RESET 30




// receiver pins:
#define rx_pin 23


//*************************** Constants end *********************************


//*************************** Variables begin *********************************


Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

uint8_t buf[VW_MAX_MESSAGE_LEN];
uint8_t buflen = VW_MAX_MESSAGE_LEN;
String buf_string;
int command_hesh; 
int current_comand;
bool training_mode = false;

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

void setup()   {      


            
  // Initialise the IO and ISR
//  vw_set_ptt_inverted(true); // Required for DR3100
  vw_setup(2000); // Bits per sec
 
  vw_rx_start(); // Start the receiver PLL running
  pinMode(rx_pin,INPUT);
  vw_set_rx_pin(rx_pin);

  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC);
  // init done

  
  // Clear the buffer.
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.display();


/*  display.println("Hello world");
  display.display();
  delay(500);
*/

// setup for motor driver

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

// change PWM (http://forum.arduino.cc/index.php?topic=72092.0)
  int myEraser = 7;      // this is 111 in binary and is used as an eraser
  TCCR3B &= ~myEraser;   // this operation (AND plus NOT),  set the three bits in TCCR2B to 0
  TCCR4B &= ~myEraser;   // this operation (AND plus NOT),  set the three bits in TCCR2B to 0
  
  // now that CS02, CS01, CS00  are clear, we write on them a new value:
  int myPrescaler = 2;         // this could be a number in [1 , 6]. In this case, 3 corresponds in binary to 011.   
  TCCR3B |= myPrescaler;  // this operation (OR), replaces the last three bits in TCCR2B with our new value 011  
  TCCR4B |= myPrescaler;  // this operation (OR), replaces the last three bits in TCCR2B with our new value 011  
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
          delay(200);
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
  }
}  

//*********************** Loop end **********************************



