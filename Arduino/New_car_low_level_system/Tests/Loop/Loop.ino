//#include <D:\Robot\GitHub\Smooth_motion_of_the_car\Arduino\New_car_low_level_system\yEd_code_files\Library\carMotionLowLevel.h>

#include <D:\Robot\GitHub\Smooth_motion_of_the_car\Arduino\New_car_low_level_system\yEd_code_files\Library\carMotionLowLevel.cpp>

#include <StandardCplusplus.h>   // It is needed to work with the container vector (and other c++ ) in the Arduino

#include <vector>






carMotionLowLevel car;


std::vector <carMotionLowLevel::Motor> vector_of_motors2, vector_of_motors;
carMotionLowLevel::Motor motor_temp, motor1, motor2;

byte time_to_change_1_PWM;

void setup() 
{
    Serial.begin(115200);
    
    // setup motor driver pins
    
    motor1.motor_driver_pinA_id = def_motor1_driver_pinA_id;
    motor1.motor_driver_pinB_id = def_motor1_driver_pinB_id;
    motor1.motor_driver_PWM_pin_id = def_motor1_driver_PWM_pin_id;

    motor2.motor_driver_pinA_id = def_motor2_driver_pinA_id;
    motor2.motor_driver_pinB_id = def_motor2_driver_pinB_id;
    motor2.motor_driver_PWM_pin_id = def_motor2_driver_PWM_pin_id;    

    
    pinMode (motor1.motor_driver_pinA_id, OUTPUT);
    pinMode (motor1.motor_driver_pinB_id, OUTPUT);
    pinMode (motor1.motor_driver_PWM_pin_id, OUTPUT);
    
    pinMode (motor2.motor_driver_pinA_id, OUTPUT);
    pinMode (motor2.motor_driver_pinB_id, OUTPUT);
    pinMode (motor2.motor_driver_PWM_pin_id, OUTPUT);    
    
    
    time_to_change_1_PWM = 100; // milliseconds
    motor1.time_previous_PWM_change = millis();
    motor1.target_PWM = 100;
    motor2.time_previous_PWM_change = millis();
    motor2.target_PWM = 100;
    
    vector_of_motors.push_back(motor1);
    vector_of_motors.push_back(motor2);    
}

void loop()
{
    car.movementUpdate(&vector_of_motors, time_to_change_1_PWM);
    Serial.println(vector_of_motors[0].current_PWM);
    Serial.println(vector_of_motors[1].current_PWM);
    delay(5);
    Serial.println("Next");
}