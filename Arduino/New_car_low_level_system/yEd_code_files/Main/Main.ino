#include <E:\Robot\GitHub\Smooth_movement_of_the_Car\Arduino\New_car_low_level_system\yEd_code_files\Library\carMotionLowLevel.h>
#include <E:\Robot\GitHub\Smooth_movement_of_the_car\Arduino\New_car_low_level_system\yEd_code_files\Library\carMotionLowLevel.cpp>


radioStation radio;
carDisplay carDisp;
Motor left_motor;
Machine_room machine_room;

void setup()
{
	Serial.begin(115200);
	pinMode(13, OUTPUT);
	
	carDisp.displayInit();
	carDisp.displayMessage("Ready!");
	
	radio.radioInit();
	
	left_motor.motorInit(def_motor_left_driver_pinA_id,
			def_motor_left_driver_pinB_id,
			def_motor_left_driver_PWM_pin_id,
			def_max_PWM);
	
	
}



void loop()
{
	radio.recieveCommand ();
	decryptCommand();
	
/*	
	target_PWM_right,left = model.calculate_and_tell_target_PWMs(radio.tell_v_target_of_center(), radio.tell_w_target(), monitoring.tell_ dv_dPWM_of_mpu());
	

	right_motor.set_new_PWM_to_current_PWM(machine_room.calculate_new_PWM(right_motor.tell_current_PWM(), right_motor.tell_t_PWM_was_set(), target_PWM_right));
	
	left_motor.set_new_PWM_to_current_PWM(machine_room.calculate_new_PWM(left_motor.tell_current_PWM(), left_motor.tell_t_PWM_was_set(), target_PWM_left));
	
*/	
	
	
	
	
	if (radio.recieveCommand ())
	{
		carDisp.displayMessage(radio.tell_command_message());
		if (radio.tell_command_message() == "8") {left_motor.set_new_PWM_to_current_PWM(-245);}
		else {left_motor.set_new_PWM_to_current_PWM(1);}
	}
	
	carDisp.displayMessage(String(left_motor.tell_current_PWM()));
	carDisp.displayMessage(String(left_motor.tell_t_PWM_was_set()));
	delay(500);
	carDisp.displayClear();

}

