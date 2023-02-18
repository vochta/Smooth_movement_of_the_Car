#include <E:\Robot\GitHub\Smooth_movement_of_the_Car\Arduino\New_car_low_level_system\yEd_code_files\Library\carMotionLowLevel.h>
#include <E:\Robot\GitHub\Smooth_movement_of_the_car\Arduino\New_car_low_level_system\yEd_code_files\Library\carMotionLowLevel.cpp>


radioStation radio;
carDisplay carDisp;
Machine_room machine_room;
Motion_model_of_the_car model;
Monitoring_feedback_circuit monitoring;


void setup()
{
	Serial.begin(115200);
	pinMode(13, OUTPUT);
	
	carDisp.displayInit();
	carDisp.printlnMessage("Ready!");
	
	radio.radioInit();
	machine_room.machine_roomInit();
	monitoring.monitoringInit();
}



void loop()
{
	radio.recieveCommand ();
/*	carDisp.printlnMessage(String(radio.tell_command_message()));
	carDisp.printMessage(String(radio.tell_v_target_of_center()));
	carDisp.printlnMessage(String(radio.tell_w_target()));
	
	carDisp.printMessage(String(machine_room.left_motor.tell_current_PWM()));
	carDisp.printlnMessage(String(machine_room.right_motor.tell_current_PWM()));
	carDisp.printMessage(String(model.tell_target_PWM_left()));			
	carDisp.printMessage(String(model.tell_target_PWM_right()));		
//	delay(500);
	carDisp.displayClear();
*/
//	carDisp.displayClear();
//	carDisp.printlnMessage(String(millis()));
	
	model.calculate_new_target_PWMs(radio.tell_v_target_of_center(), radio.tell_w_target(), monitoring.tell_EST_dv_dPWM_of_mpu_Kalman());
	model.update_new_target_PWMs_for_swing();
	
	machine_room.left_motor.set_new_PWM_to_current_PWM(machine_room.calculate_new_PWM(machine_room.left_motor.tell_current_PWM(), machine_room.left_motor.tell_t_PWM_was_set(), model.tell_target_PWM_left()));
	
	machine_room.right_motor.set_new_PWM_to_current_PWM(machine_room.calculate_new_PWM(machine_room.right_motor.tell_current_PWM(), machine_room.right_motor.tell_t_PWM_was_set(), model.tell_target_PWM_right()));
	
	
	monitoring.calculate_new_dv_dPWM_of_mpu(model.calculate_PWM_of_mpu(machine_room.right_motor.tell_current_PWM(), machine_room.left_motor.tell_current_PWM()));
	
	
	// не правильно вызывать калмана в каждом цикле, тк нет еще sum_v и соответственно нет dv_dPWM и Калман просто уменьшает ошибку ожидания, необоснованно и бесполезно
//	monitoring.calculate_new_EST_dv_dPWM_of_mpu_Kalman(radio.tell_v_target_of_center());
	
	
}

