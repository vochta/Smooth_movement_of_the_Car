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
//MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

//*************************** Includes end *********************************




//*************************** Constants begin *********************
#define def_max_PWM 255.0
#define def_time_for_commands_execution_from_stop_state 2000; // миллисекунды
#define def_duration_t_mes_inst_velocity 400; ; // миллисекунды


// Begin motors driver (Monster shield) pins
#define def_motor_left_driver_pinA_id 7
#define def_motor_left_driver_pinB_id 8
#define def_motor_left_driver_PWM_pin_id 5

#define def_motor_right_driver_pinA_id 4
#define def_motor_right_driver_pinB_id 9
#define def_motor_right_driver_PWM_pin_id 6


/*
#define PWM2 6
#define EN1 A0
#define EN2 A1
#define CS1 A2
#define CS2 A3
*/
// End motors driver (Monster shield) pins

// Begin reciever pins
#define def_good_message_recieved_pin 12	// LED pin
#define def_rx_pin 23						// data pin

// End reciever pins


// Begin LCD pins

#define def_OLED_CLK 25
#define def_OLED_MOSI 24
#define def_OLED_CS 26
#define def_OLED_DC 28
#define def_OLED_RESET 30

// End LCD pins

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

// ******** End MPU6050 ******************

//*************************** Constants end ************************


class radioStation
{
		static const char good_message_recieved_pin = def_good_message_recieved_pin; // LED pin
		static const char rx_pin = def_rx_pin; // data pin
		String command_message = ""; 

		int v_target_of_center = 0;
		int w_target_of_car = 0;

		void decryptCommand();
	
	public:
		void radioInit();
		char recieveCommand();
		
		String tell_command_message();
		int tell_v_target_of_center();
		int tell_w_target();
};

class carDisplay
{
		// LCD pins:
		static const int8_t OLED_CLK = def_OLED_CLK;
		static const int8_t OLED_MOSI = def_OLED_MOSI;
		static const int8_t OLED_CS = def_OLED_CS;
		static const int8_t OLED_DC = def_OLED_DC;
		static const int8_t OLED_RESET = def_OLED_RESET;

		Adafruit_SSD1306 car_display = {OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS};
	
	public:
		void displayInit();
		void displayClear();
		void printMessage(String message);
		void printlnMessage(String message);
};


class Motor
{                
		byte motor_driver_pinA_id;
		byte motor_driver_pinB_id;
		byte motor_driver_PWM_pin_id;
		float max_PWM;
		
		float current_PWM = 0.0;
		unsigned long t_PWM_was_set = 0;
		
		void store_t_current_PWM_was_set();
		void set_current_PWM_to_pins();
		void powerToMotor(bool motor_driver_pinA_value, bool motor_driver_pinB_value, int PWM_pin_value);
		
	public:
		void motorInit(const byte pinA,
			const byte pinB,
			const byte PWM_pin,
			const byte maxPWM);
		void set_new_PWM_to_current_PWM(float new_PWM);
		float tell_current_PWM();
		unsigned long tell_t_PWM_was_set();
};


class Machine_room
{
		static const int time_for_commands_execution_from_stop_state = def_time_for_commands_execution_from_stop_state;
				

		void change_arduino_PWM_frequancy_to_4000 ();
		
	public:
		Motor right_motor, left_motor;
		void machine_roomInit();
		float calculate_new_PWM(float current_PWM, unsigned long t_PWM_was_set, float target_PWM);
};

class Motion_model_of_the_car
{		
		static const byte d_beatween_wheels = 20;  // 20cm
		float target_PWM_left = 0.0;
		float target_PWM_right = 0.0;
		float PWM_of_center = 0.0;
		float PWM_of_mpu = 0.0;

	public:
		void calculate_new_target_PWMs(int v, int w, float dv_dPWM);
		float tell_target_PWM_left();
		float tell_target_PWM_right();
		void calculate_PWM_of_center(float PWM_right, float PWM_left);
		float tell_PWM_of_center();
		float calculate_PWM_of_mpu(float PWM_right, float PWM_left);
		float tell_PWM_of_mpu();
};		

class Mpu_values
{
		MPU6050 mpu;
		// MPU control/status vars
		uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
		uint16_t fifoCount;     // count of all bytes currently in FIFO
		uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
		uint8_t fifoBuffer[64]; // FIFO storage buffer
		uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
		// orientation/motion vars
		Quaternion q;           // [w, x, y, z]         quaternion container
		VectorInt16 aa;         // [x, y, z]            accel sensor measurements
		VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
		VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
		VectorFloat gravity;    // [x, y, z]            gravity vector
		VectorInt16 aaReal_previous; 
		
		float t_new_acc_was_gotten = 0.0;
		float t_previous_acc_was_gotten = 0.0;
		
			
		
	public:
		void mpu_init();
		void getAccelerationsValues();
		float tell_acc_y();
		float tell_acc_y_previous();
		float tell_t_new_acc_was_gotten();
		float tell_t_previous_acc_was_gotten();
};


class Instantaneous_velocity_calculator
{
		Mpu_values mpu_val;
		float inst_velocity = 0.0;
		float sum_velocity = 0.0;

	public:
		void velocity_calculatorInit();
		void calculate_inst_velocity();
		void sum_inst_velocity();
		float tell_sum_velocity();
		void clear_sum_velocity();
};	
		
class Monitoring_feedback_circuit
{
		const byte duration_t_mes_inst_velocity = def_duration_t_mes_inst_velocity;
		float dv_dPWM_of_mpu = 3.0/70.0; //def start
		float PWM_mpu_start;
		unsigned long t_mes_start;
		float v_mes_start;
		Instantaneous_velocity_calculator v_calculator;
		
	public:
		void monitoringInit();
		void calculate_new_dv_dPWM_of_mpu(float PWM_of_mpu);
		float tell_dv_dPWM_of_mpu();
};




#endif