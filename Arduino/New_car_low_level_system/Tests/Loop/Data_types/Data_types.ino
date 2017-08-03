#include <StandardCplusplus.h>

#include <iostream>
#include <vector>

class Motor
{
public:
  int id;
  String name;
  int PWM_pin_current_value;
  int PWM_pin_target_value;
};

std::vector <Motor> vector_of_motors;
Motor motor_temp;

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
   for (std::vector<Motor>::iterator it = vector_of_motors.begin() ; it != vector_of_motors.end(); ++it)
    
//for (int it = 0 ; it != vector_of_motors.size(); ++it)
    {
        Serial.print(it->name);
        Serial.println(it->id);
      //  Serial.println(vector_of_motors[*it].id);
    //    Serial.println(vector_of_motors[it].name);
        //Serial.println(*it->id);
    }
    
    delay(5000);
    Serial.println("Next");
}