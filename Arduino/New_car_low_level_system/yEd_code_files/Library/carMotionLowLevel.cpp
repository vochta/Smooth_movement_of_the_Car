#include "Arduino.h"
#include <D:\Robot\GitHub\Smooth_motion_of_the_car\Arduino\New_car_low_level_system\yEd_code_files\Library\carMotionLowLevel.h>
#include <StandardCplusplus.h>
#include <vector>

// Begin Machine_room (movement_command_execution)

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




// остановился здесь - задется не ускорение максимальное и минимлаьное, а скорость!!! посмотри в уже обкатанном файле.
// Нет, похоже эту функцию нужно отменить, упразднить - нам думаю не интересен желаемый конечный ШИМ конкретной команды..
// Или нет, в этой функции вообще речь не об изменении ШИМ или ускорения или скорости, это просто таблица. И функция вроде доделана..))
// Смотри в тетрадке , проработай дальнейший алгоритм использования этих желаемых скоростей,  ШИМов и тд.  причем за любую единицу времени

//Желаемые, соотвествующие желаемой команде, значениея датчиков у нас фиксированы в рамках данной программы. Это просто таблица соответствия, не меняется, не вычисляется
carMotionLowLevel::SensorsValues carMotionLowLevel::movementCommandConvertingToSensorsValues(byte motors_plateform_type, byte movement_command)
{
    carMotionLowLevel::SensorsValues sensors_values;

    switch (command_hesh)
    {
        case (8):         // forward
        //или  это значения датчика  от stop к forward? Но тогда как быть если была команда стоп до этого, но времени с ее поступления прошло мало и машинка не полностью остановилась, то есть по факту она не в состоянии стоп...
        // И еще у меня же пружины - акселерометр будет колбасить постоянно. Может быть его нужно на нижнюю палубу крепить? С другой стороны н,ижнюю палубу ведь тоже колбасит - не даром же я амортизацию затеял)) То есть, возможно там все суммируется и вычитается,  и в итоге получается итоговое, правильное ускорение за взятый интервал времени.
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

// End Machine_room (movement_command_execution)


/*
// Begin Monitoring / feedback circuit

// начал писать...
inline void getAccelerationsValues()
{
	mpuIntStatus = mpu.getIntStatus();
   
	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
	{
		// reset so we can continue cleanly
		mpu.resetFIFO();
		Serial.println(F("FIFO overflow!"));

		
	// otherwise, check for DMP data ready interrupt (this should happen frequently)
	} 
	else if (mpuIntStatus & 0x02) 
	{
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);
		
		// get real acceleration, adjusted to remove gravity
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetAccel(&aa, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
	/*  Serial.print("areal\t");
		Serial.print(aaReal.x);
		Serial.print("\t");
		Serial.println(aaReal.y);
		Serial.print("\t");
		Serial.println(aaReal.z);
	*/
	
	/*
	}       
	return aaReal; // ????
}

inline void getVelosity()
{
	aaReal = getAccelerationsValues;
	t_current = millis();
	Vy_current += (t_current-t_old)*(aaReal.y+accY_old)/2000;
	t_old = t_current;
	accY_old = aaReal.y;
	
	
	// Обнуление Vy_current при набравшемся значении и при условии, что вычисление скорости сейчас не ведется t_command_execution_start
//Куда это вынести?? ведь здесь оно не сработает никогда!!	
	if ((t_old > Vtimer)&&(t_command_execution_start == 0))
	{
		Vtimer = Vtimer+40000;
		Vy_current =0;
	}
}

*/



?? void v_measurement()
{
    if ((start_measurement == true)&&((millis()-t_command_execution_start) > t_measurement))
    {
        v_delta = (Vy_current-v_start);
        Serial.print("Time elapsed = ");
        Serial.println(millis()-t_command_execution_start);
        Serial.print("** = ");
        Serial.println(command_movement_direction*v_delta);

        if ((command_movement_direction*v_delta) <= 0)  // check the direction of Vy_current - is it the same with command_movement_direction? 
        {
            display.println("Wrong movement direction");
            display.display(); 
            Serial.println("Wrong movement direction");
        }

        // look v_current

        if (abs(v_delta) < v_target_min)
        {
            stop_car();
            t_go_pause = millis() + go_pause_lenght;
            display.println("To slow");
            display.print("Vdelta = ");
            display.println(v_delta);                
            display.display();
            Serial.println("To slow");
            drive_PWM += PWM_drive_change_step;
        }
        else if (abs(v_delta) > v_target_max)
        {
         //   t_go_pause = millis() + go_pause_lenght;
         //   stop_car();
            display.println("Too fast");
            display.print("Vdelta = ");
            display.println(v_delta);                
            display.display();
            Serial.println("Too fast");
            drive_PWM -= PWM_drive_change_step;
        }
        
        else 
        {
            display.println("Good");
            display.print("Vdelta = ");
            display.println(v_delta);                
            display.display();          
            Serial.println("Good");
        }

        start_measurement = false;
        t_command_execution_start = 0;
        
        Serial.print("Vdelta = ");
        Serial.println(v_delta);
        
        Serial.print("drive_PWM = ");
        Serial.println(drive_PWM);
    }
}



// End Monitoring / feedback circuit


// Begin Current state of all system (organism)

byte carMotionLowLevel::getCurrentGeneralEmotionalSystemState()
{
    return 1;  // пока просто заглушка
}

// End Current state of all system (organism)


