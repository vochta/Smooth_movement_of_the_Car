#include <E:\Robot\GitHub\Smooth_movement_of_the_Car\Arduino\New_car_low_level_system\yEd_code_files\Library\carMotionLowLevel.h>
#include <E:\Robot\GitHub\Smooth_movement_of_the_car\Arduino\New_car_low_level_system\yEd_code_files\Library\carMotionLowLevel.cpp>


radioStation radio;
carDisplay carDisp;
Machine_room machine_room;
Motion_model_of_the_car model;
Monitoring_feedback_circuit monitoring;
Mpu_values mpu;

float test_PWM = 100;

void setup()
{
	Serial.begin(115200);
	pinMode(13, OUTPUT);
	
	carDisp.displayInit();
	carDisp.printlnMessage("Ready!");
	
	radio.radioInit();
	machine_room.machine_roomInit();
	
	mpu.mpu_init();
	

}



void loop()
{



	radio.recieveCommand();
	if (radio.tell_command_message() == "8")
	{
		Serial.print("****************** command *************** - ");	
		Serial.println(millis());	
		machine_room.left_motor.set_new_PWM_to_current_PWM(test_PWM);
		machine_room.right_motor.set_new_PWM_to_current_PWM(test_PWM);
		
		while (radio.tell_command_message() != "5")
		{
			mpu.getAccelerationsValues();
			radio.recieveCommand();
		}
		machine_room.left_motor.set_new_PWM_to_current_PWM(0);
		machine_room.right_motor.set_new_PWM_to_current_PWM(0);
	}
	


	
	

}

