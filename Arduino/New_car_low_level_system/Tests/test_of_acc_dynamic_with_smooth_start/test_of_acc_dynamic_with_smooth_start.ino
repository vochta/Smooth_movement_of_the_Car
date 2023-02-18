#include <E:\Robot\GitHub\Smooth_movement_of_the_Car\Arduino\New_car_low_level_system\yEd_code_files\Library\carMotionLowLevel.h>
#include <E:\Robot\GitHub\Smooth_movement_of_the_car\Arduino\New_car_low_level_system\yEd_code_files\Library\carMotionLowLevel.cpp>


Mpu_values mpu;
Machine_room machine_room;
Motion_model_of_the_car model;

int PWM = 20;
int i;

float data[400][3]; //время, acc, PWM

void setup()
{
	Serial.begin(115200);

	mpu.mpu_init();
	machine_room.machine_roomInit();
	
	
	delay(150000);
}



void loop()
{
	for (i=0;i<20;i++)
	{
		while (mpu.getAccelerationsValues()!=1){}
		data[i][0] = mpu.tell_t_new_acc_was_gotten();
		data[i][1] = mpu.tell_acc_y();
		data[i][2] = machine_room.left_motor.tell_current_PWM();

	}
//	machine_room.left_motor.set_new_PWM_to_current_PWM(0);
//	machine_room.right_motor.set_new_PWM_to_current_PWM(0);


		machine_room.left_motor.set_new_PWM_to_current_PWM(0);
		machine_room.right_motor.set_new_PWM_to_current_PWM(0);

	for (i=20;i<400;i++)
	{
		//Serial.println(machine_room.left_motor.tell_current_PWM());
		machine_room.left_motor.set_new_PWM_to_current_PWM(machine_room.calculate_new_PWM(machine_room.left_motor.tell_current_PWM(), machine_room.left_motor.tell_t_PWM_was_set(), PWM));
	
		machine_room.right_motor.set_new_PWM_to_current_PWM(machine_room.calculate_new_PWM(machine_room.right_motor.tell_current_PWM(), machine_room.right_motor.tell_t_PWM_was_set(), PWM));
		
		while (mpu.getAccelerationsValues()!=1){}
		mpu.getAccelerationsValues();
		data[i][0] = mpu.tell_t_new_acc_was_gotten();
		data[i][1] = mpu.tell_acc_y();
		data[i][2] = machine_room.left_motor.tell_current_PWM();

	}

	machine_room.left_motor.set_new_PWM_to_current_PWM(0);
	machine_room.right_motor.set_new_PWM_to_current_PWM(0);
	

	if (Serial)
	{
		Serial.print("PWM = ");
		Serial.print(PWM);
		Serial.println("    ****************** start ***************");
		for (i=0;i<400;i++)
		{
			
			Serial.print(data[i][0]);
			Serial.print(" / ");
			Serial.print(data[i][1]);
			Serial.print(" / ");	
			Serial.println(data[i][2]);
		}
	}

	while (Serial.available() <= 0){}
	Serial.read();
}

