#include <D:\Robot\GitHub\Smooth_motion_of_the_car\Arduino\New_car_low_level_system\yEd_code_files\Library\carMotionLowLevel.h>

#include <StandardCplusplus.h>   // It is needed to work with the container vector (and other c++ ) in the Arduino

#include <vector>

carMotionLowLevel car;


std::vector <carMotionLowLevel::Motor> vector_of_motors;
carMotionLowLevel::Motor motor_temp;

void setup() 
{
    motor_temp.id = 53;
    motor_temp.name = "name1";
    vector_of_motors.push_back(motor_temp);
    motor_temp.id = -698;
    motor_temp.name = "name2";    
    vector_of_motors.push_back(motor_temp);    
    Serial.begin(115200);
}

void loop()
{
    car.movementUpdate(vector_of_motors);
    
    delay(5000);
    Serial.println("Next");
}