#ifndef carMotionLowLevel_h
#define carMotionLowLevel_h
#include "Arduino.h"
#include <StandardCplusplus.h>
#include <vector>

//*************************** Constants begin *********************
#define max_PWM 255


// Begin motors driver (Monster shield) pins
#define def_motor1_driver_pinA_id 7
#define def_motor1_driver_pinB_id 8
#define def_motor1_driver_PWM_pin_id 5

#define def_motor2_driver_pinA_id 4
#define def_motor2_driver_pinB_id 9
#define def_motor2_driver_PWM_pin_id 6


/*
#define PWM2 6
#define EN1 A0
#define EN2 A1
#define CS1 A2
#define CS2 A3
*/
// End motors driver (Monster shield) pins


// Begin system states
#define calm_state 1
#define exited_state 2
// End system states



//*************************** Constants end ************************


class carMotionLowLevel  {
    public:
        class Motor
        {
            public:
                byte motor_id;
                String motor_name;                
                byte motor_driver_pinA_id;
                byte motor_driver_pinB_id;
                byte motor_driver_PWM_pin_id;
                byte motor_driver_pinA_current_value = 0;
                byte motor_driver_pinB_current_value = 1;
                byte motor_driver_PWM_pin_current_value = 0;
                byte motor_driver_PWM_pin_target_value = 0;
                int current_PWM = 0;
                int target_PWM =0;
                unsigned long time_previous_PWM_change = 0;
              //  unsigned long time_to_change_1_PWM   ??? надо ли его делать для каждого мотора отдельным...
                float current_ampere_value;
                float shaft_rotation_speed_value;
        };

        class MotorDriverPins
        {
            public:
                class Ids
                {
                    public:
                        byte motor_driver_pinA_id; 
                        byte motor_driver_pinB_id; 
                        byte motor_driver_PWM_pin_id;
                };
                class Values
                {
                    public:
                        byte motor_driver_pinA_value; 
                        byte motor_driver_pinB_value;               
                        byte PWM_pin_value;
                };                

        };
        
        class SensorsValues
        {
            public:
                class Mpu
                {
                    public:
                        // min and max values of accelerometer sensor MPU6050
                        char min;
                        char max;
                };
            
        };


        //...
        inline void movementUpdate(std::vector <carMotionLowLevel::Motor> vector_of_motors);

        
        
        inline carMotionLowLevel::MotorDriverPins::Values translateNumberToPwmPins(int new_PWM);
        
        inline void PwmUpdate (byte motor_driver_pinA_id, byte motor_driver_pinB_id, byte motor_driver_PWM_pin_id, int new_PWM);
        
        inline void PowerToMotor (byte motor_driver_pinA_id, byte motor_driver_pinB_id, byte motor_driver_PWM_pin_id, byte motor_driver_pinA_value, byte motor_driver_pinB_value, byte PWM_pin_value);
        
        int newPwmCalculate (int current_PWM, int target_PWM, byte time_to_change_1_PWM, unsigned long *time_previous_PWM_change);
        
        void carMotionLowLevel::movementUpdate(std::vector <carMotionLowLevel::Motor> *vector_of_motors, byte time_to_change_1_PWM);
        
        byte carMotionLowLevel::getCurrentGeneralEmotionalSystemState();
        
};
#endif