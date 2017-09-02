#ifndef carMotionLowLevel_h
#define carMotionLowLevel_h


//*************************** Includes begin *********************************
#include "Arduino.h"
#include <StandardCplusplus.h>
#include <vector>

#include <SPI.h>
#include <VirtualWire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

//*************************** Includes end *********************************




//*************************** Constants begin *********************
#define max_PWM 255


// Begin motors driver (Monster shield) pins
#define def_motor1_driver_pinA_id 7
#define def_motor1_driver_pinB_id 8
#define def_motor1_driver_PWM_pin_id 5

#define def_motor2_driver_pinA_id 4
#define def_motor2_driver_pinB_id 9
#define def_motor2_driver_PWM_pin_id 6
#define def_dv_dPWM_min 1
#define def_dv_dPWM_max 2

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
						
						// границы: минимальное и максимальное изменение скорости по отношению к изменению ШИМ 
						float dv_dPWM_min;
						float dv_dPWM_max;
                };
            
        };

		
		class Velosity {
			public:
				float v_x = 0;
				float v_y = 0;
				float v_z = 0;	
				float v_start_y = 0;
				float v_delta_y = 0;
				float acc_y_old;
				unsigned long t_current, t_old, Vtimer, t_command_execution_start = 0, t_measurement = 400;
				VectorInt16 getAccelerationsValues();
				void getVelosityChange(VectorInt16 aaReal);
				void v_measurement(float command_movement_direction, float v_target_min, float v_target_max);
		};
				
				
        //...
        inline void movementUpdate_test1(std::vector <carMotionLowLevel::Motor> vector_of_motors);

        
        
        inline carMotionLowLevel::MotorDriverPins::Values translateNumberToPwmPins(int new_PWM);
        
		
        inline void PwmUpdate (byte motor_driver_pinA_id, byte motor_driver_pinB_id, byte motor_driver_PWM_pin_id, int new_PWM);
        
		
        inline void PowerToMotor (byte motor_driver_pinA_id, byte motor_driver_pinB_id, byte motor_driver_PWM_pin_id, byte motor_driver_pinA_value, byte motor_driver_pinB_value, byte PWM_pin_value);
        
		
        int newPwmCalculate (int current_PWM, int target_PWM, byte time_to_change_1_PWM, unsigned long *time_previous_PWM_change);
        
		
        void carMotionLowLevel::movementUpdate(std::vector <carMotionLowLevel::Motor> *vector_of_motors, byte time_to_change_1_PWM);
        
		
        byte carMotionLowLevel::getCurrentGeneralEmotionalSystemState();
		
		
		carMotionLowLevel::SensorsValues carMotionLowLevel::movementCommandConvertingToSensorsValues(byte motors_plateform_type, byte movement_command);
        
};
#endif