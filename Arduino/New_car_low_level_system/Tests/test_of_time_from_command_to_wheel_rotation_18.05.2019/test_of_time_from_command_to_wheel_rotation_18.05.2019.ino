#include <E:\Robot\GitHub\Smooth_movement_of_the_Car\Arduino\New_car_low_level_system\yEd_code_files\Library\carMotionLowLevel.h>
#include <E:\Robot\GitHub\Smooth_movement_of_the_car\Arduino\New_car_low_level_system\yEd_code_files\Library\carMotionLowLevel.cpp>


radioStation radio;
carDisplay carDisp;
Machine_room machine_room;
Motion_model_of_the_car model;
Monitoring_feedback_circuit monitoring;
Mpu_values mpu;

float test_PWM = 255;

String command = "5";
int n = 10000;   // количество обращений за показаниями асс,   в реальности показаний будет меньше в 4-5 раз, тк обращения быстрее чем время готовности одного нового показания

void setup()
{
	Serial.begin(230400);
	pinMode(13, OUTPUT);
	
	carDisp.displayInit();
	carDisp.printlnMessage("Ready!");
	
	radio.radioInit();
	machine_room.machine_roomInit();
	
	mpu.mpu_init();



}



void loop()
{


	while (command != "8")
	{
		if (radio.recieveCommand())
		{
			command = radio.tell_command_message();
		}
	}
	
	for (int i=0; i<n; i++)  
	{
		mpu.getAccelerationsValues();
	}
	
	Serial.println("go");	
//	Serial.println(millis());
	machine_room.left_motor.set_new_PWM_to_current_PWM(test_PWM);
	machine_room.right_motor.set_new_PWM_to_current_PWM(test_PWM);
	
	
	for (int i=0; i<n; i++) 
	{
		mpu.getAccelerationsValues();
	}
	Serial.print("left_motor.tell_t_PWM_was_set: ");
	Serial.println(machine_room.left_motor.tell_t_PWM_was_set());
	Serial.print("right_motor.tell_t_PWM_was_set: ");
	Serial.println(machine_room.right_motor.tell_t_PWM_was_set());
	

	
	machine_room.left_motor.set_new_PWM_to_current_PWM(0);
	machine_room.right_motor.set_new_PWM_to_current_PWM(0);
	
	Serial.println("stop");	
	for (int i=0; i<n; i++)  
	{
		mpu.getAccelerationsValues();
	}
	
	command = "5";


	
	

}

