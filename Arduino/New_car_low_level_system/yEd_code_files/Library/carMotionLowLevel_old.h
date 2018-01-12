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
#define def_dv_dPWM_min 0.1
#define def_dv_dPWM_max 0.2
#define def_dv_dPWM_forward 0.15
#define def_coeff_dv_dPWM 500
#define def_time_to_change_1_PWM 33 // миллисекунды. 3 секунды на команду.

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

// ******** Begin MPU6050 ******************
// Arduino --> MPU6050 pins:
    // Arduino Nano:
    // A4 --> SDA
    // A5 --> SCL
    // D2 --> INT (interrupt, if needed)
    
    // Arduino Mega:
    // A20 --> SDA
    // A21 --> SCL
    // D2 --> INT (interrupt, if needed)
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

// ******** End MPU6050 ******************

//*************************** Constants end ************************


class carMotionLowLevel  {
    public:
		float dv_dPWM_forward; // = def_dv_dPWM_forward;
		float coeff_dv_dPWM; // = def_coeff_dv_dPWM;
		float time_to_change_1_PWM;
		int t_exe;
		char v_target;
		
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
						float dv_dPWM;
                };
            
        };

		
		class Velosity {
			public:
				float dv_x = 0;
				float dv_y = 0;
				float dv_z = 0;	
				float v_start_y = 0;
				float v_delta_y = 0;
				float acc_y_old;
				unsigned long t_current, t_old, Vtimer, t_command_execution_start = 0, t_measurement = 400;

				// MPU control/status vars
				uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
				uint16_t fifoCount;     // count of all bytes currently in FIFO
				uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
				uint8_t fifoBuffer[64]; // FIFO storage buffer

				// orientation/motion vars
				Quaternion q;           // [w, x, y, z]         quaternion container
				VectorInt16 aa;         // [x, y, z]            accel sensor measurements
				VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
				VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
				VectorFloat gravity;    // [x, y, z]            gravity vector

				VectorInt16 getAccelerationsValues();
				int old_PWM;
				int delta_PWM;
				void getVelosityChange(VectorInt16 aaReal);
				float v_check(float command_movement_direction, float v_target_min, float v_target_max, float dv_dPWM);
				void v_monitor(int current_PWM, float dv_dPWM_min, float dv_dPWM_max);
		};

        
        
        inline carMotionLowLevel::MotorDriverPins::Values translateNumberToPwmPins(int new_PWM);
        
		
        inline void PwmUpdate (byte motor_driver_pinA_id, byte motor_driver_pinB_id, byte motor_driver_PWM_pin_id, int new_PWM);
        
		
        inline void PowerToMotor (byte motor_driver_pinA_id, byte motor_driver_pinB_id, byte motor_driver_PWM_pin_id, byte motor_driver_pinA_value, byte motor_driver_pinB_value, byte PWM_pin_value);
        
		
        int newPwmCalculate (int current_PWM, int target_PWM, float time_to_change_1_PWM, unsigned long *time_previous_PWM_change);
        
		
        void carMotionLowLevel::movementUpdate(std::vector <carMotionLowLevel::Motor> *vector_of_motors);
        
		
        byte carMotionLowLevel::getCurrentGeneralEmotionalSystemState();
		
		
		carMotionLowLevel::SensorsValues::Mpu carMotionLowLevel::movementCommandConvertingToSensorsValues(byte motors_plateform_type, byte movement_command);
		
		void carMotionLowLevel::Sensors_values_converting_to_motors_values (carMotionLowLevel::SensorsValues::Mpu *mpu_values, std::vector <carMotionLowLevel::Motor> *vector_of_motors);
		
		void make_PWM_correction(float PWM_correction_coeff, std::vector <carMotionLowLevel::Motor> *vector_of_motors);
        
		void init (carMotionLowLevel car);
};
#endif