#include <VirtualWire.h>

void setup()
{
	vw_setup(2000); 						// Bits per sec
	Serial.begin(9600);
}
 
void loop()
{
	if (Serial.available()) 				// check if data has been sent from the computer
	{
		char command = Serial.read();
		digitalWrite(13, true);			 	// Flash a light to show transmitting
		vw_send((uint8_t *)&command, 1);  	// Тип данных требуется ссылка на byte. Передача по одному символу. 
		vw_wait_tx();						// Wait until the whole message is gone
		delay(50);
		digitalWrite(13, false);
	}
}