 
#include <VirtualWire.h>

void setup()
{
  // Initialise the IO and ISR
  //vw_set_ptt_inverted(true);    // Required for DR3100
  vw_setup(2000); // Bits per sec

  // transmitter VCC
  pinMode(6, OUTPUT);
  digitalWrite(6, HIGH);

  Serial.begin(9600);   // initialize the serial communication
}
 
void loop()
{
  if (Serial.available()) // check if data has been sent from the computer
  {
    char command = Serial.read();

    digitalWrite(13, true); // Flash a light to show transmitting
    vw_send((uint8_t *)&command, 1);  //strlen(&command));
    vw_wait_tx(); // Wait until the whole message is gone
    digitalWrite(13, false);
    delay(50);
  }
}
