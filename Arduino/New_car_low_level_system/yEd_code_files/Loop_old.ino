#include <D:\Robot\GitHub\Smooth_movement_of_the_Car\Arduino\New_car_low_level_system\yEd_code_files\Library\carMotionLowLevel.h>
#include <D:\Robot\GitHub\Smooth_movement_of_the_car\Arduino\New_car_low_level_system\yEd_code_files\Library\carMotionLowLevel.cpp>

#include <StandardCplusplus.h>   // It is needed to work with the container vector (and other c++ ) in the Arduino

#include <vector>

carMotionLowLevel car;


std::vector <carMotionLowLevel::Motor> vector_of_motors;
carMotionLowLevel::Motor motor_temp;
carMotionLowLevel::SensorsValues::Mpu mpu_values;
carMotionLowLevel::Velosity v; // переменная для работы со скоростью, ускорением

/*
Эта переменная влияет на скорость выполненния команды в целом, 
тк время выполнения команды, например, из состояния покоя это целевой ШИМ поноженный на эту переменную.
То есть, как бы скорость выполнения самого маленького, самого элементарного действия.
Нужно будет, наверное, потом завязать эту переменную на эмоциональное состояние системы.
*/

byte movement_command = 8; // для тестов задаю команду движения явным образом, вперед

uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)

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
	
	
    //********** Begin setup accelerometer *********
    
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
		v.packetSize = mpu.dmpGetFIFOPacketSize();
        Serial.print("PacketSize = ");
        Serial.print(v.packetSize);
		
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
	
	car.init (&car);
}




// забить обновление шима на моторрах




void loop()
{
	mpu_values = car.movementCommandConvertingToSensorsValues(1, movement_command);
/*	car.Sensors_values_converting_to_motors_values(&mpu_values, &vector_of_motors);
/*	
	Serial.print("dv_dPWM_min1 = ");
	Serial.println(mpu_values.dv_dPWM_min);	
	Serial.println(mpu_values.dv_dPWM_max);
	
	for (std::vector<carMotionLowLevel::Motor>::iterator motor = vector_of_motors.begin(); motor != vector_of_motors.end(); ++motor)
	{
		Serial.println(motor->motor_name);
		Serial.print("target_PWM = ");
		Serial.println(motor->target_PWM);
		Serial.print("current_PWM = ");
		Serial.println(motor->current_PWM);		
	}
	
	//Serial.println(motor_temp.target_PWM);
    delay(1);
	*//*
	car.movementUpdate(&vector_of_motors);
	v.v_monitor(vector_of_motors[0].current_PWM, mpu_values.dv_dPWM_min, mpu_values.dv_dPWM_max);
	//Serial.println(v.dv_y);
 //   Serial.println("Next");
 
 */
}