#include <D:\Robot\GitHub\Smooth_movement_of_the_Car\Arduino\New_car_low_level_system\yEd_code_files\Library\carMotionLowLevel.h>
#include <D:\Robot\GitHub\Smooth_movement_of_the_car\Arduino\New_car_low_level_system\yEd_code_files\Library\carMotionLowLevel.cpp>

#include <StandardCplusplus.h>   // It is needed to work with the container vector (and other c++ ) in the Arduino

#include <vector>

carMotionLowLevel car;


std::vector <carMotionLowLevel::Motor> vector_of_motors;
carMotionLowLevel::Motor motor_temp;
byte time_to_change_1_PWM = 1/120;

carMotionLowLevel::Velosity v; // переменная для работы со скоростью, ускорением

void setup() 
{
    motor_temp.motor_id = 1;
    motor_temp.motor_name = "Left front motor";
	motor_temp.target_PWM =100;
	
	motor_temp.motor_driver_pinA_id = def_motor1_driver_pinA_id;
	motor_temp.motor_driver_pinB_id = def_motor1_driver_pinB_id;
	motor_temp.motor_driver_PWM_pin_id =  def_motor1_driver_PWM_pin_id;
    vector_of_motors.push_back(motor_temp);
	
    motor_temp.motor_id = 3;
    motor_temp.motor_name = "Right back motor";   
	motor_temp.target_PWM =200;	
	motor_temp.motor_driver_pinA_id = def_motor2_driver_pinA_id;
	motor_temp.motor_driver_pinB_id = def_motor2_driver_pinB_id;
	motor_temp.motor_driver_PWM_pin_id = def_motor2_driver_PWM_pin_id;
    vector_of_motors.push_back(motor_temp); 
	
	
    Serial.begin(115200);
}

void loop()
{
    car.movementUpdate(&vector_of_motors, time_to_change_1_PWM);
    v.getVelosityChange(v.getAccelerationsValues());
	
    delay(5000);
	Serial.println(v.v_y);
    Serial.println("Next");
}