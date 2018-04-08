#include "Arduino.h"
#include <StandardCplusplus.h>
#include <vector>


void radioStation::radioInit()
{
	vw_setup(2000); // Bits per sec
    vw_rx_start(); // Start the receiver PLL running
    pinMode(rx_pin,INPUT);
    vw_set_rx_pin(rx_pin);
	pinMode(good_message_recieved_pin, OUTPUT);
}

char radioStation::recieveCommand()
{
	uint8_t buf[VW_MAX_MESSAGE_LEN];
	uint8_t buflen = VW_MAX_MESSAGE_LEN;
	digitalWrite(good_message_recieved_pin, false);
    if (vw_get_message(buf, &buflen)) // Non-blocking
    {
        digitalWrite(good_message_recieved_pin, true); // Flash a light to show received good message
        // Message with a good checksum received, dump it.

        // get message
        command_message = ""; 
        for (int i = 0; i < buflen; i++)
        {
          command_message += char(buf[i]);
        }
		decryptCommand();
		return (1);
    }
	return (0);
}

String radioStation::tell_command_message()
{
	return command_message;
}

int radioStation::tell_v_target_of_center()
{
	return v_target_of_center;
}

int radioStation::tell_w_target()
{
	return w_target_of_car;
}

void radioStation::decryptCommand()
{
	if (command_message =="8" )
	{
		v_target_of_center = 3;
		w_target_of_car = 0;
	}
	else if (command_message =="2")
	{
		v_target_of_center = -3;
		w_target_of_car = 0;		
	}
	else if (command_message =="6")
	{
		v_target_of_center = 0;
		w_target_of_car = -10;		
	}	
	else if (command_message =="4")
	{
		v_target_of_center = 0;
		w_target_of_car = 10;		
	}	
}

void carDisplay::displayInit()
{
    car_display.begin(SSD1306_SWITCHCAPVCC);
    // **** init done ****
    
    // Clear display the buffer.
    car_display.setTextColor(WHITE);
	displayClear();
}

void carDisplay::displayClear()
{
        car_display.setCursor(0,0);
        car_display.clearDisplay();
		car_display.display();
}

void carDisplay::printMessage(String message)
{
	    car_display.print(message);
		car_display.print(" ");
        car_display.display();
}

void carDisplay::printlnMessage(String message)
{
	    car_display.println(message);
        car_display.display();
}

void Motor::motorInit(const byte pinA,
			const byte pinB,
			const byte PWM_pin, 
			const byte maxPWM)
{
	motor_driver_pinA_id = pinA;
	motor_driver_pinB_id = pinB;
	motor_driver_PWM_pin_id = PWM_pin;
	max_PWM = maxPWM;
}

void Motor::set_new_PWM_to_current_PWM(float new_PWM)
{
    if (abs(new_PWM) > max_PWM)
    {
		current_PWM = ((new_PWM > 0) - (new_PWM < 0))*max_PWM;
    }
	else current_PWM = new_PWM;
	set_current_PWM_to_pins();
	store_t_current_PWM_was_set();
}

float Motor::tell_current_PWM()
{
	return current_PWM;
}

void Motor::store_t_current_PWM_was_set()
{
	t_PWM_was_set = millis();
}

void Motor::set_current_PWM_to_pins()
{
	if (current_PWM >= 0)
    {
		powerToMotor(0,1,current_PWM);
    }
    else
    {
		powerToMotor(1,0, abs(current_PWM));
    }
}

void Motor::powerToMotor(bool motor_driver_pinA_value, bool motor_driver_pinB_value, int PWM_pin_value)
{
    digitalWrite(motor_driver_pinA_id, motor_driver_pinA_value);
    digitalWrite(motor_driver_pinB_id, motor_driver_pinB_value);
    analogWrite(motor_driver_PWM_pin_id, PWM_pin_value);
}

unsigned long Motor::tell_t_PWM_was_set()
{
	return t_PWM_was_set;
}

void Machine_room::machine_roomInit()
{
	change_arduino_PWM_frequancy_to_4000();
	left_motor.motorInit(def_motor_left_driver_pinA_id,
			def_motor_left_driver_pinB_id,
			def_motor_left_driver_PWM_pin_id,
			def_max_PWM);
			
	right_motor.motorInit(def_motor_right_driver_pinA_id,
			def_motor_right_driver_pinB_id,
			def_motor_right_driver_PWM_pin_id,
			def_max_PWM);
}


void Machine_room::change_arduino_PWM_frequancy_to_4000()
{
	    // change PWM from 500 Hz to 4000 Hz 
    //(http://forum.arduino.cc/index.php?topic=72092.0)
    int myEraser = 7;      // this is 111 in binary and is used as an eraser
    TCCR3B &= ~myEraser;   // this operation (AND plus NOT),  set the three bits in TCCR2B to 0
    TCCR4B &= ~myEraser;   // this operation (AND plus NOT),  set the three bits in TCCR2B to 0

    // now that CS02, CS01, CS00  are clear, we write on them a new value:
    int myPrescaler = 2;         // this could be a number in [1 , 6]. In this case, 3 corresponds in binary to 011.   
    TCCR3B |= myPrescaler;  // this operation (OR), replaces the last three bits in TCCR2B with our new value 011  
    TCCR4B |= myPrescaler;  // this operation (OR), replaces the last three bits in TCCR2B with our new value 011  
}

float Machine_room::calculate_new_PWM(float current_PWM, unsigned long t_PWM_was_set, float target_PWM)
{
	unsigned long dt = millis() - t_PWM_was_set;
    float dPWM = dt*(fabsf(target_PWM))/time_for_commands_execution_from_stop_state;

	if (debug_serial_print_1)
	{	
		Serial.print("dt: ");
		Serial.println(dt);
		Serial.print("target_PWM: ");
		Serial.println(target_PWM);	
		Serial.print("dPWM: ");
		Serial.println(dPWM);
	}
	
	if (dPWM < abs(target_PWM - current_PWM)) // если шаг dPWM_drive, который сейчас нужно будет сделать, больше, чем осталось до target_PWM, то его не делаем, а просто считаем что цель достигнута и ставим current_PWM = target_PWM.  Это чтобы ШИМ не болтало из стороны в сторону - то перепрыгнули в одну сторону, потом в другую.
	{
		if (current_PWM > target_PWM)
		{
			current_PWM -= dPWM;
		}
		else
		{
			current_PWM += dPWM;
		}
	}
	else current_PWM = target_PWM;
    return current_PWM;
}

void Motion_model_of_the_car::calculate_new_target_PWMs(int v, int w, float dv_dPWM )
{
	if (dv_dPWM == 0)    // заплатка...
	{
		dv_dPWM = 3.0/70.0;
	}
	
	if ((w==0)&&(v != 0))
	{
		target_PWM_right = v / dv_dPWM;   // как  оно может быть типа int ,  если его значение определяется делением?!!
		target_PWM_left = target_PWM_right;
	}		
	else if ((v==0)&&(w!=0))
	{
		target_PWM_right = 3.14*d_beatween_wheels*w/360/2/dv_dPWM;
		target_PWM_left = - target_PWM_right;
	}
	
	if (target_PWM_right == 0)
	{
		if (debug_serial_print_1)
		{
			Serial.print("v: ");
			Serial.println(v);
			
			Serial.print("dv_dPWM: ");
			Serial.println(dv_dPWM);
			
			Serial.print("target_PWM_right: ");
			Serial.println(target_PWM_right);
		}
	//	delay(3000);
	}
	
}

void Motion_model_of_the_car::calculate_PWM_of_center(float PWM_left, float PWM_right)
{
	PWM_of_center = (PWM_left + PWM_right)/2;
}

float Motion_model_of_the_car::tell_PWM_of_center()
{
	return PWM_of_center;
}

float Motion_model_of_the_car::calculate_PWM_of_mpu(float PWM_left, float PWM_right)
{
	PWM_of_mpu = (PWM_left + PWM_right)/2;
	return PWM_of_mpu;
}

float Motion_model_of_the_car::tell_PWM_of_mpu()
{
	return PWM_of_mpu;
}

float Motion_model_of_the_car::tell_target_PWM_left()
{
	return target_PWM_left;
}

float Motion_model_of_the_car::tell_target_PWM_right()
{
	return target_PWM_right;
}

void Mpu_values::mpu_init()
{
	aaReal.y = 0;
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize device MPU6050
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    
    // set offsets of this particular MPU6050
    mpu.setXGyroOffset(25);
    mpu.setYGyroOffset(45);
    mpu.setZGyroOffset(44);
    mpu.setXAccelOffset(-5858);
    mpu.setYAccelOffset(1349);
    mpu.setZAccelOffset(1190);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

 /*       // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
   */     
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        Serial.print("PacketSize = ");
        Serial.print(packetSize);
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    //********** End setup accelerometer *********	
}

float Mpu_values::tell_acc_y()
{
	aaReal_previous = aaReal;
	getAccelerationsValues();
	return aaReal.y;
}

float Mpu_values::tell_acc_y_previous()
{
	return aaReal_previous.y;
}

float Mpu_values::tell_t_new_acc_was_gotten()
{
	return t_new_acc_was_gotten;
}

float Mpu_values::tell_t_previous_acc_was_gotten()
{
	return t_previous_acc_was_gotten;
}

void Mpu_values::getAccelerationsValues()
{
	mpuIntStatus = mpu.getIntStatus();
   
	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
	{
		// reset so we can continue cleanly
		mpu.resetFIFO();
		if (debug_serial_print_1)
		{	
			Serial.println(F("FIFO overflow!"));
		}
		
	// otherwise, check for DMP data ready interrupt (this should happen frequently)
	} 
	else if (mpuIntStatus & 0x02) 
	{
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
		t_previous_acc_was_gotten = t_new_acc_was_gotten;
		t_new_acc_was_gotten = millis();
		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);
		
		// get real acceleration, adjusted to remove gravity
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetAccel(&aa, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
/*		Serial.print("areal\t");
		Serial.print(aaReal.x);
		Serial.print("\t");
		
*/		if (debug_serial_print_vchart)
		{
		/*	Serial.println("accY");
			Serial.println(aaReal.y);
			Serial.println(millis());
			*/
		}
		
/*		Serial.println(aaReal.z);
*/
	}       
}

void Instantaneous_velocity_calculator::velocity_calculatorInit()
{
	mpu_val.mpu_init();
}

float Instantaneous_velocity_calculator::tell_sum_velocity()
{
	return sum_velocity;
}


void Instantaneous_velocity_calculator::clear_sum_velocity()
{
	sum_velocity = 0;
}

void Instantaneous_velocity_calculator::calculate_inst_velocity()
{
	inst_velocity = (mpu_val.tell_t_new_acc_was_gotten()-mpu_val.tell_t_previous_acc_was_gotten())*(mpu_val.tell_acc_y()+mpu_val.tell_acc_y_previous())/2000;  // 2000 - это  на сколько помню просто коэффициент перевода данных акселлерометра в м/с или см/с...
}

void Instantaneous_velocity_calculator::sum_inst_velocity()
{
	sum_velocity += inst_velocity;
}

void Monitoring_feedback_circuit::monitoringInit()
{
	v_calculator.velocity_calculatorInit();
}

void Monitoring_feedback_circuit::calculate_new_dv_dPWM_of_mpu(float PWM_of_mpu)
{
	v_calculator.calculate_inst_velocity();
	v_calculator.sum_inst_velocity();
	if (debug_serial_print_1)
	{
		Serial.print("sum vel: ");
		Serial.println(v_calculator.tell_sum_velocity());
	}
	
	if ((millis() - t_mes_start) > duration_t_mes_inst_velocity)
	{
		if (debug_serial_print_1)
		{
			Serial.print("dv_dPWM_of_mpu: ");
			Serial.println(dv_dPWM_of_mpu);
			Serial.print("PWM_of_mpu: ");
			Serial.println(PWM_of_mpu);
		}
		
		if (debug_serial_print_vchart)
		{		
			if (v_calculator.tell_sum_velocity()>200) Serial.println("************************************");
			Serial.println("sumv");
			Serial.println(v_calculator.tell_sum_velocity());
			Serial.println(millis());//debug_points_counter++);
			
		}	
		
		float dPWM = (PWM_of_mpu - PWM_mpu_start);
		if (dPWM!=0)
		{
			dv_dPWM_of_mpu = v_calculator.tell_sum_velocity()/dPWM;
			PWM_mpu_start = PWM_of_mpu;
		}
		v_calculator.clear_sum_velocity();
		t_mes_start = millis();
	}
}

float Monitoring_feedback_circuit::tell_dv_dPWM_of_mpu()
{
	return dv_dPWM_of_mpu;
}