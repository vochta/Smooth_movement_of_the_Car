#include "Arduino.h"
#include <D:\Robot\GitHub\Smooth_motion_of_the_car\Arduino\New_car_low_level_system\yEd_code_files\Library\carMotionLowLevel.h>
#include <StandardCplusplus.h>
#include <vector>



//...
inline void carMotionLowLevel::movementUpdate(std::vector <carMotionLowLevel::Motor> vector_of_motors)
{
    for (std::vector<carMotionLowLevel::Motor>::iterator it = vector_of_motors.begin() ; it != vector_of_motors.end(); ++it)
    {
        Serial.println(it->motor_name);
        Serial.println(it->motor_id);
    }        
};


inline void carMotionLowLevel::PowerToMotor (byte motor_driver_pinA_id, byte motor_driver_pinB_id, byte motor_driver_PWM_pin_id, byte motor_driver_pinA_value, byte motor_driver_pinB_value, byte PWM_pin_value)
{
    digitalWrite(motor_driver_pinA_id, motor_driver_pinA_value);
    digitalWrite(motor_driver_pinB_id, motor_driver_pinB_value);
    analogWrite(motor_driver_PWM_pin_id, PWM_pin_value);
}

/*
MovementCommandExecution(movement_command)
{
    getCurrentGeneralEmotionalSystemState();
    Command_converting_to_sensors_values
    Sensors_values_converting_to_motors_values
    movementUpdate(motors);
}
*/

//Функция изменяет значение PWM каждого из переданных ей моторов
//в соответствии с переданным шагом ШИМ, прошедшим интервалом времени 
//с момента последнего изменения и заданным конечным ШИМ.
void carMotionLowLevel::movementUpdate(std::vector <carMotionLowLevel::Motor> *vector_of_motors, byte time_to_change_1_PWM)
{
    for (std::vector<carMotionLowLevel::Motor>::iterator motor = vector_of_motors->begin(); motor != vector_of_motors->end(); ++motor)
    {  
        motor->current_PWM = newPwmCalculate (motor->current_PWM, motor->target_PWM, time_to_change_1_PWM, &motor->time_previous_PWM_change);
        PwmUpdate(motor->motor_driver_pinA_id, motor->motor_driver_pinB_id, motor->motor_driver_PWM_pin_id, motor->current_PWM);
    }
}



// Вычисление нового ШИМ одного мотора. Возвращает новый ШИМ.
int carMotionLowLevel::newPwmCalculate (int current_PWM, int target_PWM, byte time_to_change_1_PWM, unsigned long *time_previous_PWM_change)
{
    unsigned long dt = millis() - *time_previous_PWM_change;
    int dPWM_drive = dt / time_to_change_1_PWM;
    if (dPWM_drive > 1) *time_previous_PWM_change += dt; 
    if (current_PWM > target_PWM)
    {
        current_PWM -= dPWM_drive;
    }
    else
    {
        current_PWM += dPWM_drive;
    }
    
    if (abs(current_PWM)>max_PWM) current_PWM = target_PWM;
 
    return current_PWM;
}


inline carMotionLowLevel::MotorDriverPins::Values carMotionLowLevel::translateNumberToPwmPins(int new_PWM)
{
    carMotionLowLevel::MotorDriverPins::Values values;
    if (new_PWM >= 0)
    {
        values.motor_driver_pinA_value = 0;
        values.motor_driver_pinB_value = 1;
    }
    else
    {
        values.motor_driver_pinA_value = 1;
        values.motor_driver_pinB_value = 0;                
    }
    
    values.PWM_pin_value = abs(new_PWM);
    if (values.PWM_pin_value > max_PWM)
    {
        values.PWM_pin_value = max_PWM;
    }
    return values;
}

inline void carMotionLowLevel::PwmUpdate (byte motor_driver_pinA_id, byte motor_driver_pinB_id, byte motor_driver_PWM_pin_id, int new_PWM)
{
    carMotionLowLevel::MotorDriverPins::Values values;
    values = carMotionLowLevel::translateNumberToPwmPins(new_PWM);
    PowerToMotor(motor_driver_pinA_id, motor_driver_pinB_id, motor_driver_PWM_pin_id, values.motor_driver_pinA_value, values.motor_driver_pinB_value, values.PWM_pin_value);
}


byte carMotionLowLevel::getCurrentGeneralEmotionalSystemState()
{
    return 1;
}


// ОСТАНОВИЛСЯ ЗДЕСЬ

//Желаемые, соотвествующие желаемой команде, значениея датчиков у нас фиксированы в рамках данной программы. Это просто таблица соответствия, не меняется, не вычисляется
carMotionLowLevel::SensorsValues carMotionLowLevel::movementCommandConvertingToSensorsValues(byte motors_plateform_type, byte movement_command)
{
    carMotionLowLevel::SensorsValues sensors_values;

    switch (command_hesh)
    {
        case (8):         // forward   
        //или  это значения датчика  от stop к forward? Но тогда как быть если была команда стоп до этого, но времени с ее поступления прошло мало и машинка не полностью остановилась, то есть по факту она не в состоянии стоп...
        
            sensors_values.Mpu.min = 10;
            sensors_values.Mpu.max = 30;            
        break;

        case (2):         // backward
            sensors_values.Mpu.min = -30;
            sensors_values.Mpu.max = -10; 
        break;

        case (4):         // turn left

        break;

        case (6):       //  turn right

        break;      
    }        
}



// возвращает команду? а если сбой при получении? наверное внутри 
/*char MovementCommandReceive()
{
    
}
*/



