#include <E:\Robot\GitHub\Smooth_movement_of_the_Car\Arduino\New_car_low_level_system\yEd_code_files\Library\carMotionLowLevel.h>
#include <E:\Robot\GitHub\Smooth_movement_of_the_car\Arduino\New_car_low_level_system\yEd_code_files\Library\carMotionLowLevel.cpp>

#define start_PWM 30
#define koeff_PWM 1.5



Mpu_values mpu;
Machine_room machine_room;
Motion_model_of_the_car model;

int PWM = start_PWM;
int i, i_left_encoderPin_state_changed, i_right_encoderPin_state_changed, i_max_PWM;
int left_encoderPin = 35;
int right_encoderPin = 37;
bool new_left_encoderPin_state, old_left_encoderPin_state;
bool new_right_encoderPin_state, old_right_encoderPin_state;


float data[400][3]; //время, acc, PWM

// float lag_from_command_to_wheels = 100, PWM_to_move;

float S_positive=0.0, S_negative=0.0, S_neg_pos = 0.9, time_right_encoderPin_state_changed = 0, time_left_encoderPin_state_changed = 0;


void setup()
{
	Serial.begin(115200);

	mpu.mpu_init();
	machine_room.machine_roomInit();
	
	old_left_encoderPin_state = digitalRead(left_encoderPin);
	old_right_encoderPin_state = digitalRead(right_encoderPin);
	
	delay(100000);
}



void loop()
{

	S_positive = 0;
	S_negative = 0;
	
	
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


		if (time_left_encoderPin_state_changed == 0)
		{
			new_left_encoderPin_state = digitalRead(left_encoderPin);
			if (old_left_encoderPin_state != new_left_encoderPin_state)
			{
				time_left_encoderPin_state_changed = millis();
				old_left_encoderPin_state = new_left_encoderPin_state;
				i_left_encoderPin_state_changed = i;
				if (PWM == start_PWM) PWM = PWM*koeff_PWM;
			}
		}

		
		if (time_right_encoderPin_state_changed == 0)
		{
			new_right_encoderPin_state = digitalRead(right_encoderPin);
			if (old_right_encoderPin_state != new_right_encoderPin_state)
			{
				time_right_encoderPin_state_changed = millis();
				old_right_encoderPin_state = new_right_encoderPin_state;
				i_right_encoderPin_state_changed = i;
				if (PWM == start_PWM) PWM = PWM*koeff_PWM;
			}
		}
		
		if (machine_room.left_motor.tell_current_PWM() != PWM) i_max_PWM = i; 
		
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
			
			Serial.print(data[i][0],0);
			Serial.print(" / ");
			Serial.print(data[i][1],0);
			Serial.print(" / ");	
			Serial.println(data[i][2]);
		}
	}

	for (i= (max(i_left_encoderPin_state_changed,i_right_encoderPin_state_changed));i<=i_max_PWM;i++)
	{
		if ((data[i][1]+data[i-1][1])>0)
		{
			S_positive += ((data[i][1]+data[i-1][1])/2)*((data[i][0]-data[i-1][0]));
		}
		else if (((data[i][1]+data[i-1][1])<0))
		{
			S_negative += ((data[i][1]+data[i-1][1])/2)*((data[i-1][0]-data[i][0]));
		}
		
	}
	
	Serial.println("S_positive = " + String(S_positive, 0));
	Serial.println("S_negative = " + String(S_negative, 0));
	Serial.print("time_left_encoderPin_state_changed = ");
	Serial.println(time_left_encoderPin_state_changed, 0);
	Serial.print("time_right_encoderPin_state_changed = ");
	Serial.println(time_right_encoderPin_state_changed, 0);
	Serial.print("i_left_encoderPin_state_changed = ");
	Serial.println(i_left_encoderPin_state_changed);
	Serial.print("i_right_encoderPin_state_changed = ");
	Serial.println(i_right_encoderPin_state_changed);
	Serial.print("i_max_PWM = ");
	Serial.println(i_max_PWM);	
	
	
	while (Serial.available() <= 0){}
	Serial.read();
	
	PWM = 20;
	old_left_encoderPin_state = digitalRead(left_encoderPin);
	old_right_encoderPin_state = digitalRead(right_encoderPin);
	S_positive=0.0; S_negative=0.0; S_neg_pos = 0.9; 
	time_left_encoderPin_state_changed = 0; 
	time_right_encoderPin_state_changed = 0;
	i_left_encoderPin_state_changed = 0;
	i_right_encoderPin_state_changed = 0;
	i_max_PWM = start_PWM;
}

