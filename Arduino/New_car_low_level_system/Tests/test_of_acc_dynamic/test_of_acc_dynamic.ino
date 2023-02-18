#include <E:\Robot\GitHub\Smooth_movement_of_the_Car\Arduino\New_car_low_level_system\yEd_code_files\Library\carMotionLowLevel.h>
#include <E:\Robot\GitHub\Smooth_movement_of_the_car\Arduino\New_car_low_level_system\yEd_code_files\Library\carMotionLowLevel.cpp>


Mpu_values mpu;
Machine_room machine_room;

int PWM = 40;
int i;

float data[500][3]; //время, acc, PWM

void setup()
{
	Serial.begin(115200);

	mpu.mpu_init();
	machine_room.machine_roomInit();
	
	

}



void loop()
{
	delay(100000);
	machine_room.left_motor.set_new_PWM_to_current_PWM(PWM);
	machine_room.right_motor.set_new_PWM_to_current_PWM(PWM);	
	delay(5000);
	
	for (i=0;i<20;i++)
	{
		while (mpu.getAccelerationsValues()!=1){}
	//	mpu.getAccelerationsValues();
		data[i][0] = mpu.tell_t_new_acc_was_gotten();
		data[i][1] = mpu.tell_acc_y();
	//	data[i][2] = PWM;
		//delay(2);
	}
	machine_room.left_motor.set_new_PWM_to_current_PWM(0);
	machine_room.right_motor.set_new_PWM_to_current_PWM(0);

	for (i=20;i<200;i++)
	{
		while (mpu.getAccelerationsValues()!=1){}
		mpu.getAccelerationsValues();
		data[i][0] = mpu.tell_t_new_acc_was_gotten();
		data[i][1] = mpu.tell_acc_y();
	//	data[i][2] = PWM;
		//delay(2);
	}

	machine_room.left_motor.set_new_PWM_to_current_PWM(0);
	machine_room.right_motor.set_new_PWM_to_current_PWM(0);
	
	while(1)
	{
		if (Serial)
		{
			Serial.print("PWM = ");
			Serial.print(PWM);
			Serial.println("    ****************** start ***************");
			for (i=0;i<200;i++)
			{
				
				Serial.print(data[i][0]);
				Serial.print(" / ");
				Serial.println(data[i][1]);
				//Serial.print(" / ");	
				//Serial.println(data[i][2]);
			}
		}	
		delay(-1);
	}
	
	
}

