#include "Arduino.h"
//#include <D:\Robot\GitHub\Smooth_movement_of_the_car\Arduino\New_car_low_level_system\yEd_code_files\Library\carMotionLowLevel.h>
#include <StandardCplusplus.h>
#include <vector>

// Begin Machine_room (movement_command_execution)

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
void carMotionLowLevel::movementUpdate(std::vector <carMotionLowLevel::Motor> *vector_of_motors)
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
    if (dPWM_drive > 1)
	{
		*time_previous_PWM_change += dt; // а если dPWM_drive меньше единицы, то есть мы не можем изменить ШИМ на целое число, то мы его не меняем, а также не меняем time_previous_PWM_change и в следующий раз интервал прошедшего времени будет считаться с учетом этого раза, то есть он будет точно больше и dPWM_drive увеличится. Так дойдем до целого dPWM_drive.
	
		if (dPWM_drive < abs(target_PWM - current_PWM)) // если шаг dPWM_drive, который сейчас нужно будет сделать, больше, чем осталось до target_PWM, то его не делаем, а просто считаем что цель достигнута и ставим current_PWM = target_PWM.  Это чтобы ШИМ не болтало из стороны в сторону - то перепрыгнули в одну сторону, потом в другую.
		{
			if (current_PWM > target_PWM)
			{
				current_PWM -= dPWM_drive;
			}
			else
			{
				current_PWM += dPWM_drive;
			}
		}
		else current_PWM = target_PWM;
	}
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

// возвращает команду? а если сбой при получении? наверное внутри
/*char MovementCommandReceive()
{

}
*/

// End Machine_room (movement_command_execution)


// Begin Monitoring / feedback circuit


// задется не ускорение максимальное и минимлаьное, а скорость!!! посмотри в уже обкатанном файле.
// Нет, похоже эту функцию нужно отменить, упразднить - нам думаю не интересен желаемый конечный ШИМ конкретной команды..
// Или нет, в этой функции вообще речь не об изменении ШИМ или ускорения или скорости, это просто таблица. И функция вроде доделана..))
// Смотри в тетрадке , проработай дальнейший алгоритм использования этих желаемых скоростей,  ШИМов и тд.  причем за любую единицу времени


//Желаемые, соотвествующие желаемой команде, значениея датчиков у нас фиксированы в рамках данной программы. Это просто таблица соответствия, не меняется, не вычисляется
carMotionLowLevel::SensorsValues::Mpu carMotionLowLevel::movementCommandConvertingToSensorsValues(byte motors_plateform_type, byte movement_command)
{
    carMotionLowLevel::SensorsValues::Mpu mpu_values;

    switch (movement_command)
    {
        case (8):         // forward
        //или  это значения датчика  от stop к forward? Но тогда как быть если была команда стоп до этого, но времени с ее поступления прошло мало и машинка не полностью остановилась, то есть по факту она не в состоянии стоп...
        // И еще у меня же пружины - акселерометр будет колбасить постоянно. Может быть его нужно на нижнюю палубу крепить? С другой стороны н,ижнюю палубу ведь тоже колбасит - не даром же я амортизацию затеял)) То есть, возможно там все суммируется и вычитается,  и в итоге получается итоговое, правильное ускорение за взятый интервал времени.
    /*        mpu_values.dv_dPWM_min = def_dv_dPWM_min;  
            mpu_values.dv_dPWM_max = def_dv_dPWM_max;
	*/
		mpu_values.dv_dPWM = dv_dPWM_forward;
		v_target = 2;
        break;

        case (2):         // backward
            mpu_values.dv_dPWM_min = -def_dv_dPWM_min;
            mpu_values.dv_dPWM_max = -def_dv_dPWM_max;
        break;

        case (4):         // turn left

        break;

        case (6):       //  turn right

        break;
    }
	
	return mpu_values;
}


// Таблица соответствия ощущений и значений параметров машинного отделения.
// желаемые значения датчиков (ощущения) <--> PWM
/* А вот значения для машинного отделения, соответствующие желаемым значениям датчиков, 
могут вполне конкурировать между собой, тк машинное отделение одно и все датчики могут от него зависить.
То есть, для достижения значений одного датчика может требоваться поставить на определенном моторе один ШИМ, 
а для достижения значений другого датчика на этом же моторе может требоваться, по этой таблице, поставить другой ШИМ.
Соотвествтенно, если действительно рассматривать систему с несколькими датчиками связанными с одним, например, мотором,
то нужно разрешить вопрос о конкуренции этих датчиков. А вернее вопрос приоритетов или веса действия значений этих 
датчиков при решении, что делать исходя из показаний этих датчиков. 

А от одно ли мотора зависят показания разных датчиков? может у нас и нет пересекающихся, если внимательно посмотреть.

Наверное, мне нужно для одно датчика ускорения все это сделать для начала. Это скорее будет отдельная программа, не общее
решения для многих датчиков, но зато можно будет работать дальше в направлении обучения объектам.
*/

void carMotionLowLevel::Sensors_values_converting_to_motors_values (carMotionLowLevel::SensorsValues::Mpu *mpu_values, std::vector <carMotionLowLevel::Motor> *vector_of_motors)
{
    for (std::vector<carMotionLowLevel::Motor>::iterator motor = vector_of_motors->begin(); motor != vector_of_motors->end(); ++motor)
    {
		motor->target_PWM = (mpu_values->dv_dPWM)*coeff_dv_dPWM;
    //	motor->target_PWM = ((mpu_values->dv_dPWM_min + mpu_values->dv_dPWM_max)/2)*500; // цель - среднее значение ШИМ. Далее подгон под конкретные моторы, покрытие пола, напряжение питания.. например мне нужен ШИМ 75 из 255 для медленного движения.
		if (abs(motor->target_PWM)>max_PWM) motor->target_PWM = max_PWM; 
		/* 
		защита от назначения целевого ШИМ выходящего за пределы физически возможного значения ШИМ 
		для данного устройства.  max_PWM - это пока константа, назначенная в заголовочном файле, одна для всех моторов.
		*/
/*		Serial.print("target_PWM = ");
		Serial.println(motor->target_PWM);	
*/		
    }
}


VectorInt16 carMotionLowLevel::Velosity::getAccelerationsValues()
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
/*		Serial.print("areal\t");
		Serial.print(aaReal.x);
		Serial.print("\t");
		Serial.println(aaReal.y);
		Serial.print("\t");
		Serial.println(aaReal.z);
*/
	}       
	return aaReal; 
}

inline void carMotionLowLevel::Velosity::getVelosityChange(VectorInt16 aaReal)
{
	t_current = millis();
	dv_y += (t_current-t_old)*(aaReal.y+acc_y_old)/2000;
	t_old = t_current;
	acc_y_old = aaReal.y;
}	
/*	
	// Обнуление Vy_current при набравшемся значении и при условии, что вычисление скорости сейчас не ведется t_command_execution_start
//Куда это вынести?? ведь здесь оно не сработает никогда!!	
	if ((t_old > Vtimer)&&(t_command_execution_start == 0))
	{
		Vtimer = Vtimer+40000;
		Vy_current =0;
	}
}

*/

float carMotionLowLevel::Velosity::v_check(float command_movement_direction,float v_target_min, float v_target_max, float dv_dPWM)
{
	dv_dPWM = (dv_dPWM*2 + dv_y/delta_PWM)/3;
	//time_to_change_1_PWM = t_exe*dv_dPWM/v_target;
	

	Serial.print("Time elapsed = ");
	Serial.println(millis()-t_command_execution_start);


	if ((command_movement_direction*dv_y) <= 0)  // check the direction of Vy_current - is it the same with command_movement_direction? 
	{
/*           display.println("Wrong movement direction");
		display.display(); 
*/          Serial.println("Wrong movement direction");
	}

 //   start_measurement = false;
	t_command_execution_start = 0;
//	v_start_y = 0;
	acc_y_old = 0;
	dv_y = 0;
//	mpu.resetFIFO();
	
	Serial.print("Vdelta = ");
	Serial.println(v_delta_y);
	
//	Serial.print("drive_PWM = ");
   // Serial.println(drive_PWM);
   
   return PWM_correction_coeff;
}

void make_PWM_correction(float PWM_correction_coeff, std::vector <carMotionLowLevel::Motor> *vector_of_motors)
{
	for (std::vector<carMotionLowLevel::Motor>::iterator motor = vector_of_motors->begin(); motor != vector_of_motors->end(); ++motor)
    {
    	motor->target_PWM = PWM_correction_coeff*motor->target_PWM;
	}
}


void carMotionLowLevel::Velosity::v_monitor(int current_PWM, float dv_dPWM_min, float dv_dPWM_max, float dv_dPWM)
{
	if (t_command_execution_start == 0) 
	{
		t_command_execution_start = millis();
		old_PWM = current_PWM;
	}
	else if ((millis()-t_command_execution_start) > t_measurement)
	{
		delta_PWM = current_PWM - old_PWM;
		float v_target_min = delta_PWM*dv_dPWM_min;
		float v_target_max = delta_PWM*dv_dPWM_max;
		float command_movement_direction = (delta_PWM > 0) - (delta_PWM < 0);
		Serial.print("old_PWM = ");
		Serial.println(old_PWM);
		Serial.print("delta_PWM = ");
		Serial.println(delta_PWM);
		Serial.print("command_movement_direction = ");
		Serial.println(command_movement_direction);
		
		PWM_correction_coeff = v_check(command_movement_direction, v_target_min, v_target_max, dv_dPWM);
		make_PWM_correction(float PWM_correction_coeff, std::vector <carMotionLowLevel::Motor> *vector_of_motors);
	}
	else
	{
		getVelosityChange(getAccelerationsValues());
	}
}

// End Monitoring/feedback circuit


// Begin Current state of all system (organism)

byte carMotionLowLevel::getCurrentGeneralEmotionalSystemState()
{
    return 1;  // пока просто заглушка
}

// End Current state of all system (organism)

void init (carMotionLowLevel *car)
{
	car->time_to_change_1_PWM = def_time_to_change_1_PWM;
	car->t_exe = 3000;
}


