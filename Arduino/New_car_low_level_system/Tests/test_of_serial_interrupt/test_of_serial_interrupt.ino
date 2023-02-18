void setup()
{
	Serial.begin(115200);
				Serial.println("****************** start *************** - ");	
}

void loop()
{
	delay(1000);
	if (Serial)
	{
		Serial.println(millis());
		delay(1000);
	}
}