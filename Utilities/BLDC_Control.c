#include "BLDC_Control.h"
#include "math.h"
#include "MultivariateFilter.h"
#include "usb_lib.h"
#include <stdbool.h>

#define ABS(x)         (x < 0) ? (-x) : xS
#define RadToDeg                   (int)  57295
#define DegToRad                   (float) 0.01745

float fNormAcc, fNormAcc_initial;//,fSinRoll,fCosRoll,fSinPitch,fCosPitch = 0.0f, RollAng = 0.0f, PitchAng = 0.0f;
float altitude = 0;
float Buffer[3] = {0.0f}, AccBuffer2[3] = {0.0f}, Heading[1] = {0.0f};
float AccBuffer3[3] = {0.0f};
float MagBuffer2[3] = {0.0f};
float Initial_ACC_Z;
float LastZerror;
int Air_Pressure = 0;
int Last_Air_Pressure = 0;
int Initial_Air_Pressure = 0;
uint16_t Reset_Gyro = 0;
int throttle = 0;
int throttle_stick = 0;
int Offset = 7000;
int offsetA = 7000;
int offsetB = 7000;
int offsetC = 7000;
int offsetD = 7000;
int throttle_limiter = 0;
int offsetA_High, offsetB_High, offsetC_High, offsetD_High;
int offsetA_Low, offsetB_Low, offsetC_Low, offsetD_Low;
int duty_cycleC, duty_cycleB, duty_cycleA, duty_cycleD;
float XSum_Of_Gyro, YSum_Of_Gyro, ZSum_Of_Gyro;
float XTotal_Rotation, YTotal_Rotation, ZTotal_Rotation;
float XKalman_Gyro, XKalman_Accel, YKalman_Gyro, YKalman_Accel;
float XInitial_Rotation = 0.0;
float YInitial_Rotation = 0.0;
float ZInitial_Rotation = 0.0;
float XPrevious_Samples[5] = {0.0};
float YPrevious_Samples[5] = {0.0};
float AccXangle, AccYangle, AccZangle, heading;
float chasetheX = 0.0;//positional setpoint
float chasetheY = 0.0;//positional setpoint
float Yaw = 0.0;
float SUMof_XError = 0;
float SUMof_YError = 0;
float SUMof_ZError = 0;
float SUM_of_pitch = 0;
float SUM_of_roll = 0;
float SUM_of_yaw = 0;
float XLastError = 0;
float YLastError = 0;
float XLastSlope = 0;
float YLastSlope = 0;
int Last_Throttle = 0;
float ControlX_Out = 0;
float ControlY_Out = 0;
float ControlZ_Out = 0;
int pwm_period;
int ms_pulses2;
int prescaler2;
int interrupt_frequency = 10;
int interrupt_period_int;
bool XWindup_Limit = false;
bool YWindup_Limit = false;
KalmanFilterTypeDef XaxisGYRO, XaxisACCEL, YaxisGYRO, YaxisACCEL, Zaxis;
RollingAverageTypeDef Xaverage, Yaverage;
//Set up the timer and schedule interruptions
uint8_t Motor2 = 1;
uint8_t Motor3 = 2;
uint8_t Motor4 = 3;
uint8_t Motor1 = 4;
float pitch1;
float roll1;
float alt = 0;
float accXnorm;
float accYnorm;
float heading=0;

void schedule_PI_interrupts()
{

	//cortexm4f_enable_fpu();
	interrupt_period_int = (1/(float)interrupt_frequency)*1000;
	int clk = 36e6; // 36MHz -> system core clock. This is default on the stm32f3 discovery
	/*
	 * prescaler = ((clk/interrupt_frequency)/interrupt_frequency) - 1
	 * This equation returns the PID interrupt frequency configuration value for TIM_Prescaler
	 */

	int prescaler = ((clk / interrupt_frequency)/interrupt_frequency)-1;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	pwm_period = slow_init_pwm(700);
	TIM_TimeBaseInitTypeDef timerInitStructure;
	timerInitStructure.TIM_Prescaler = prescaler;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = interrupt_frequency - 1;
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	timerInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &timerInitStructure);
	TIM_Cmd(TIM2, ENABLE);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    NVIC_InitTypeDef nvicStructure;
    nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
    nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
    nvicStructure.NVIC_IRQChannelSubPriority = 1;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure);


}
void Display_Heading(float value)
{
	uint8_t dig, i;
	float temp;
	char message[7];
	temp = value;
	dig = temp*1000;
	dig = dig %10;
	message[0] = dig+48;
	dig = temp*100;
	dig = dig %10;
	message[1] = dig+48;
	dig = temp*10;
	dig = dig %10;
	message[2] = dig+48;
	message[3] = '.';
	dig = ((uint8_t)value % 10)+48;
	message[4] = dig;
	value = value/10;
	dig = ((uint8_t)value % 10)+48;
	message[5] = dig;
	value = value/10;
	dig = ((uint8_t)value % 10)+48;
	message[6] = dig;
	for(i=7; i != 0; i=i-1)
	{
		USART1_Send(message[i-1] );
	}
	USART1_Send(0xF8);//degrees
    USART1_Send('\n');
    USART1_Send('\r');
}
void init_BT_Pair_Sense()
{


	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; // Input
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // GPIO speed - has nothing to do with the timer timing
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // Push-pull
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // Setup pull-up resistors
	GPIO_Init(GPIOA, &GPIO_InitStructure);

    //Configure Button EXTI line
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    //EXTI_Init(&EXTI_InitStructure);


    //set priotity
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
void Display_Axis(int value)
{
	int dig, i;
	int temp;
	char message[8];
	if (value < 0)
	{
		temp = value*-1;
		dig = temp %10;
		temp = temp/10;
		message[0] = dig+48;
		dig = temp %10;
		temp = temp/10;
		message[1] = dig+48;
		dig = temp %10;
		temp = temp/10;
		message[2] = dig+48;
		message[3] = '.';
		dig = temp %10;
		temp = temp/10;
		message[4] = dig+48;
		dig = temp %10;
		temp = temp/10;
		message[5] = dig+48;
		dig = temp %10;
		temp = temp/10;
		message[6] = dig+48;
		message[7] = '-';
		for(i=8; i != 0; i=i-1)
		{
			USART1_Send(message[i-1] );
		}
	}
	else {
		temp = value;
		dig = temp %10;
		temp = temp/10;
		message[0] = dig+48;
		dig = temp %10;
		temp = temp/10;
		message[1] = dig+48;
		dig = temp %10;
		temp = temp/10;
		message[2] = dig+48;
		message[3] = '.';
		dig = temp %10;
		temp = temp/10;
		message[4] = dig+48;
		dig = temp %10;
		temp = temp/10;
		message[5] = dig+48;
		dig = temp %10;
		temp = temp/10;
		message[6] = dig+48;
		for(i=7; i != 0; i=i-1)
		{
			USART1_Send(message[i-1] );
		}
	}
}
void USART1_Send_Int(float data)
{
	int value = 0;
	char byte1 = 0;
	char byte2 = 0;
	value = (int)data + 180;
	byte1 = value;
	value = (int)value>>8;
	byte2 = value;
	value = (int)value>>8;
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); // Wait for Empty
	USART_SendData(USART1, byte2);
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); // Wait for Empty
	USART_SendData(USART1, byte1);

}
void cortexm4f_enable_fpu() {
    /* set CP10 and CP11 Full Access */
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));
}

void Set_Offset(int* value, float* roll, float* pitch, int* yaw)
{
	//float xval = *value;
	throttle = 60*sqrt(*value) + 500;//*value * 0.6;
	throttle_stick = throttle;//*value;// * 0.6;

	Yaw = (float)*yaw * 0.016;
	chasetheY = (float)*roll * 0.007;
	chasetheX = (float)(0 - *pitch) * 0.007;
	offsetA_High = offsetA + 4000;
	offsetB_High = offsetB + 4000;
	offsetC_High = offsetC + 4000;
	offsetD_High = offsetD + 4000;
	offsetA_Low = offsetA - 4000;
	offsetB_Low = offsetB - 4000;
	offsetC_Low = offsetC - 4000;
	offsetD_Low = offsetD - 4000;
}
void Initialize_Position()
{
    kalmanFilter_Init(&XaxisGYRO);
    kalmanFilter_Init(&XaxisACCEL);
    kalmanFilter_Init(&YaxisGYRO);
    kalmanFilter_Init(&YaxisACCEL);
    kalmanFilter_Init(&Zaxis);
    Xaverage.Smoothing_Factor = 7;
    Yaverage.Smoothing_Factor = 7;
    XaxisGYRO.dt = 0.05;
    XaxisACCEL.dt = 0.05;
    YaxisGYRO.dt = 0.05;
    YaxisACCEL.dt = 0.05;
    Zaxis.dt = 0.05;

	float X_Pos_Sum=0.0;
	float Y_Pos_Sum=0.0;

	int i=0;
	for(i=0;i<10;i=i+1)
	{
		Calculate_Position();
		X_Pos_Sum = X_Pos_Sum + XTotal_Rotation;
		Y_Pos_Sum = Y_Pos_Sum + YTotal_Rotation;
	}
	XInitial_Rotation = X_Pos_Sum/10;
	YInitial_Rotation = Y_Pos_Sum/10;

}
void Calculate_Position()
{
	uint8_t i;
	float XGyroGain = 0, YGyroGain = 0;
	float magXcomp;
	float magYcomp;

	//float heading;
    GyroReadAngRate(Buffer);//read the angular rate from the gyroscope and store in Buffer[]
    CompassReadAcc(AccBuffer2);
    CompassReadMag(MagBuffer2);


    /*XSum_Of_Gyro = (0.02*XKalman_Accel+0.98*XKalman_Gyro) - Buffer[0]/40;
    XaxisGYRO.z = (XSum_Of_Gyro);
    XKalman_Gyro = kalmanFilter(&XaxisGYRO);
    XaxisACCEL.z = (AccBuffer2[0]/20);//57*accXnorm;//
    XKalman_Accel = kalmanFilter(&XaxisACCEL);
    XTotal_Rotation = (0.98 * XKalman_Gyro) + (0.02 * XKalman_Accel);

    YSum_Of_Gyro = (0.02*YKalman_Accel+0.98*YKalman_Gyro) - Buffer[1]/40;
    YaxisGYRO.z = (YSum_Of_Gyro);
    YKalman_Gyro = kalmanFilter(&YaxisGYRO);
    YaxisACCEL.z = (AccBuffer2[1]/20);//57*accYnorm;//
    YKalman_Accel = kalmanFilter(&YaxisACCEL);
    YTotal_Rotation = (0.98 * YKalman_Gyro) + (0.02 * YKalman_Accel);*/

    accXnorm = AccBuffer2[0]/sqrt(AccBuffer2[0]* AccBuffer2[0] + AccBuffer2[1] * AccBuffer2[1] + AccBuffer2[2] * AccBuffer2[2]);
    accYnorm = AccBuffer2[1]/sqrt(AccBuffer2[0] *AccBuffer2[0] + AccBuffer2[1] * AccBuffer2[1] + AccBuffer2[2] * AccBuffer2[2]);
    pitch1 = (0-asin(accXnorm));
    roll1 = (0-asin(accYnorm/cos(pitch1)));

    XSum_Of_Gyro = (0.03*XKalman_Accel+0.97*XKalman_Gyro) - Buffer[0]/10;
    XaxisGYRO.z = (XSum_Of_Gyro);
    XKalman_Gyro = kalmanFilter(&XaxisGYRO);
    XaxisACCEL.z = (0-(pitch1*57));//57*accXnorm;//
    XKalman_Accel = kalmanFilter(&XaxisACCEL);
    XTotal_Rotation = (0.98 * XKalman_Gyro) + (0.02 * XKalman_Accel);

    YSum_Of_Gyro = (0.03*YKalman_Accel+0.97*YKalman_Gyro) - Buffer[1]/10;
    YaxisGYRO.z = (YSum_Of_Gyro);
    YKalman_Gyro = kalmanFilter(&YaxisGYRO);
    YaxisACCEL.z = (0-(roll1*57));//57*accYnorm;//
    YKalman_Accel = kalmanFilter(&YaxisACCEL);
    YTotal_Rotation = (0.98 * YKalman_Gyro) + (0.02 * YKalman_Accel);


    //accXnorm = AccBuffer2[0]/sqrt(AccBuffer2[0]* AccBuffer2[0] + AccBuffer2[1] * AccBuffer2[1] + AccBuffer2[2] * AccBuffer2[2]);
    //accYnorm = AccBuffer2[1]/sqrt(AccBuffer2[0] *AccBuffer2[0] + AccBuffer2[1] * AccBuffer2[1] + AccBuffer2[2] * AccBuffer2[2]);
    //pitch1 = (0-asin(accXnorm));
    //roll1 = (0-asin(accYnorm/cos(pitch1)));// + 1.57;
    magXcomp = (MagBuffer2[0]*cos(pitch1))+((MagBuffer2[2])*sin(pitch1));
    magYcomp = (MagBuffer2[0]*sin(roll1)*sin(pitch1))+((0-MagBuffer2[1])*cos(roll1))-(MagBuffer2[2]*sin(roll1)*cos(pitch1));
    heading = 180*atan2(magYcomp,magXcomp)/M_PI;

    ZSum_Of_Gyro = ZSum_Of_Gyro + Buffer[2]/100;
    Zaxis.z = ZSum_Of_Gyro;
    ZTotal_Rotation = kalmanFilter(&Zaxis);

}


void Adjust_Yaw(int* value)
{
	duty_cycleC = duty_cycleC + *value;
	duty_cycleB = duty_cycleB - *value;
	duty_cycleA = duty_cycleA + *value;
	duty_cycleD = duty_cycleD - *value;
}

void bounds_check()
{
	if(duty_cycleC >= offsetC_High)
	{
		duty_cycleC = offsetC_High;
		XWindup_Limit = true;
	}
	else if(duty_cycleC <= offsetC_Low)
	{
		duty_cycleC = offsetC_Low;
		XWindup_Limit = true;
	} else {XWindup_Limit = false;}
	if(duty_cycleB >= offsetB_High)
	{
		duty_cycleB = offsetB_High;
		YWindup_Limit = true;
	}
	else if(duty_cycleB <= offsetB_Low)
	{
		duty_cycleB = offsetB_Low;
		YWindup_Limit = true;
	}else {YWindup_Limit = false;}
	if(duty_cycleA >= offsetA_High)
	{
		duty_cycleA = offsetA_High;
		XWindup_Limit = true;
	}
	else if(duty_cycleA <= offsetA_Low)
	{
		duty_cycleA = offsetA_Low;
		XWindup_Limit = true;
	}else {XWindup_Limit = false;}
	if(duty_cycleD >= offsetD_High)
	{
		duty_cycleD = offsetD_High;
		YWindup_Limit = true;
	}
	else if(duty_cycleD <= offsetD_Low)
	{
		duty_cycleD = offsetD_Low;
		YWindup_Limit = true;
	}else {YWindup_Limit = false;}
	if(duty_cycleA <= Offset)
	{
		duty_cycleA = Offset;
		XWindup_Limit = true;
	}
	if(duty_cycleB <= Offset)
	{
		duty_cycleB = Offset;
		YWindup_Limit = true;
	}
	if(duty_cycleC <= Offset)
	{
		duty_cycleC = Offset;
		XWindup_Limit = true;
	}
	if(duty_cycleD <= Offset)
	{
		duty_cycleD = Offset;
		YWindup_Limit = true;
	}
}
void init_pwm_gpio()
{
	// The Timer 1 channels 1,2 and 3 are connected to the LED pins on the Discovery board
	// To drive the ECSs they could to be connected to other pins if needed.

	// Setup the LEDs
	// Check out the Discovery schematics
	// http://www.st.com/st-web-ui/static/active/en/resource/technical/document/user_manual/DM00063382.pdf
	//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);
	GPIO_InitStructure.GPIO_Pin = LED3_PIN | LED7_PIN | LED8_PIN | LED10_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // Use the alternative pin functions
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // GPIO speed - has nothing to do with the timer timing
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // Push-pull
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // Setup pull-up resistors
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	// Connect the timer output to the LED pins
	// Check the alternative function mapping in the CPU doc
	// http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00058181.pdf
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_2); // TIM1_CH1 -> LED3
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_2); // TIM1_CH2 -> LED7
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_2); // TIM1_CH4 -> LED8
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_2); // TIM1_CH3 -> LED10
}

//  * @brief  Initializes PWM
//  * @param  pwm_freq: Frequency of the PWM in Hz
//  * @retval Number of the timer pulses per one PWM period

void init_pwm()
{

//  Enable the TIM1 peripherie
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE );

// Setup the timing and configure the TIM1 timer
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_TimeBaseStructInit(& TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = prescaler2;
	TIM_TimeBaseStructure.TIM_Period = 2860;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up ;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);


// Initialise the timer channels
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

	TIM_OCInitStructure.TIM_Pulse = ms_pulses2*2;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

	// These settings must be applied on the timer 1.
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Set;

// Setup four channels
	// Channel 1
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

	// Channel 2
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

	// Channel 3
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

	// Channel 4
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

	// Starup the timer
	TIM_ARRPreloadConfig(TIM1, DISABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	TIM_Cmd(TIM1 , ENABLE);
}
int slow_init_pwm(int pwm_freq)
{
	// Calculates the timing. This is common for all channels
	int clk = 72e6; // 72MHz -> system core clock. This is default on the stm32f3 discovery
	int tim_freq = 2e6; // in Hz (2MHz) Base frequency of the pwm timer
	int prescaler = ((clk / tim_freq) - 1);
	prescaler2 = ((clk / tim_freq) - 1);
	// Calculate the period for a given pwm frequency
	int pwm_period = tim_freq/pwm_freq;

	// Calculate a number of pulses per millisecond.
	// Not used in this routine but I put it here just as an example
	int ms_pulses = (float)pwm_period / (1000.0/pwm_freq); // for 200Hz we get: 10000 / (1/200 * 1000) = 2000
	ms_pulses2 = (float)pwm_period / (1000.0/pwm_freq);

//  Enable the TIM1 peripherie
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE );

// Setup the timing and configure the TIM1 timer
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_TimeBaseStructInit(& TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = prescaler;
	TIM_TimeBaseStructure.TIM_Period = 1857;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up ;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);


// Initialise the timer channels
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

	TIM_OCInitStructure.TIM_Pulse = ms_pulses*2;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

	// These settings must be applied on the timer 1.
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Set;

    // Setup four channels
	// Channel 1
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

	// Channel 2
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

	// Channel 3
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

	// Channel 4
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

	// Starup the timer
	TIM_ARRPreloadConfig(TIM1, DISABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	TIM_Cmd(TIM1 , ENABLE);

	return pwm_period;
}


//  * @brief  Sets the PWM duty cycle per channel
//  * @param  channel:  PWM channel index [1..4]
//  * @param  duty_cycle:  PWM duty cycle in percents (integer) [0..100]
//  * @retval None
//
void set_pwm_width(int channel, int pwm_period, uint32_t duty_cycle)//float* duty_cycle)
{
	int pwm_pulses = (pwm_period*duty_cycle)/100000;
	switch (channel){
		case 1: TIM_SetCompare1(TIM1, pwm_pulses); break;
		case 2: TIM_SetCompare2(TIM1, pwm_pulses); break;
		case 3: TIM_SetCompare3(TIM1, pwm_pulses); break;
		case 4: TIM_SetCompare4(TIM1, pwm_pulses); break;
	}
}


//  * @brief  Sets the PWM duty cycle per channel
//  * @param  channel:  PWM channel index [1..4]
//  * @param  duty_cycle:  PWM duty cycle (float) [0..1]
//  * @retval None

void set_pwm_width_norm(int channel, int pwm_period, float duty_cycle)
{
	int pwm_pulses = pwm_period*(float)duty_cycle;
	switch (channel){
		case 1: TIM_SetCompare1(TIM1, pwm_pulses); break;
		case 2: TIM_SetCompare2(TIM1, pwm_pulses); break;
		case 3: TIM_SetCompare3(TIM1, pwm_pulses); break;
		case 4: TIM_SetCompare4(TIM1, pwm_pulses); break;
	}
}


void TIM2_IRQHandler()
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
    	float Xerror = 0;
    	float Yerror = 0;
    	float Zerror = 0;
    	float SlopeofXError = 0.0;
    	float SlopeofYError = 0.0;
    	float SlopeofZError = 0.0;
    	float XGain_Scale = 0;
    	float YGain_Scale = 0;

        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

        //if ((Last_Throttle - throttle) > 40){
        //	throttle = Last_Throttle - 10;
        //}

        if (throttle_stick >= 50){


        //the difference between the current displacement and the setpoint is the error and P component
        /*if(XTotal_Rotation > 90){
        	XTotal_Rotation = 90;
        } else if(XTotal_Rotation < -90){
        	XTotal_Rotation = -90;
        }
        if(YTotal_Rotation > 90){
        	YTotal_Rotation = 90;
        } else if(YTotal_Rotation < -90){
        	YTotal_Rotation = -90;
        }*/

        Xerror = chasetheX - XTotal_Rotation;
        Yerror = chasetheY - YTotal_Rotation;
        Zerror = (0 - Yaw) + (0 - Buffer[2]);//ZTotal_Rotation/10;


        if (XTotal_Rotation > 0){
    	XGain_Scale = sin(XTotal_Rotation * DegToRad);
        } else {
        XGain_Scale = sin((0 - XTotal_Rotation) * DegToRad);
        }
        if (YTotal_Rotation > 0){
    	YGain_Scale = sin(YTotal_Rotation * DegToRad);
        } else {
        YGain_Scale = sin((0 - YTotal_Rotation) * DegToRad);
        }

        //The integral(I) component is created by multiplying the error by the period
        //and summing each individual periods error
        if ((Xerror > 1) | (Xerror < (0 - 1)))
        {
        	SUMof_XError = SUMof_XError + Xerror;
        }
        if ((Yerror > 1) | (Yerror < (0 - 1)))
        {
        	SUMof_YError = SUMof_YError + Yerror;
        }

        if ((Yaw > 0.75)|(Yaw < (0-0.75)))
        	SUMof_ZError = SUMof_ZError + Zerror;
        else {
        	SUMof_ZError = 0;
        }


        //Derivative(D) Component
        SlopeofXError = (XTotal_Rotation - XLastError);
        XLastError = XTotal_Rotation;
        SlopeofYError = (YTotal_Rotation - YLastError);
        YLastError = YTotal_Rotation;
        SlopeofZError = Buffer[2]/40;



        //We can now assemble the control output by multiplying each control component by it's associated
        //gain coefficient and summing the results

    	ControlX_Out = ((2 +(XGain_Scale * 1)) * Xerror);
    	ControlX_Out = ControlX_Out + ((0.5 +(XGain_Scale * 0.4)) * SUMof_XError);
    	ControlX_Out = ControlX_Out + ((2 +(XGain_Scale * 2.5)) * Buffer[0]);

    	ControlY_Out = ((2 +(YGain_Scale * 1)) * Yerror);
    	ControlY_Out = ControlY_Out + ((0.5 +(YGain_Scale * 0.4)) * SUMof_YError);
    	ControlY_Out = ControlY_Out + ((2 +(YGain_Scale * 2.5)) * Buffer[1]);

        ControlZ_Out = (10 * Zerror);// + (1.5 * SUMof_ZError);// + (SlopeofZError);
        }
        else{
        throttle = 0;
        SUMof_XError = 0;
        SUMof_YError = 0;
        SUMof_ZError = 0;
        ControlX_Out = 0;
        ControlY_Out = 0;
        ControlZ_Out = 0;
        //XTotal_Rotation = 0;
        //YTotal_Rotation = 0;
        //ZTotal_Rotation = 0;
        XSum_Of_Gyro = 0;
        YSum_Of_Gyro = 0;
        ZSum_Of_Gyro = 0;
        }
    	offsetA = Offset + throttle;
    	offsetB = Offset + throttle;
    	offsetC = Offset + throttle;
    	offsetD = Offset + throttle;

        duty_cycleC = (0 - ControlX_Out) + ControlY_Out + offsetC - ControlZ_Out;
        duty_cycleA = ControlX_Out + (0 - ControlY_Out) + offsetA - ControlZ_Out;
        duty_cycleD = (0 - ControlX_Out) + (0 - ControlY_Out)+ offsetD + ControlZ_Out;
        duty_cycleB = ControlX_Out + ControlY_Out + offsetB + ControlZ_Out;


        bounds_check();
        set_pwm_width(Motor2, pwm_period, duty_cycleD);
        set_pwm_width(Motor1, pwm_period, duty_cycleC);
        set_pwm_width(Motor4, pwm_period, duty_cycleB);
        set_pwm_width(Motor3, pwm_period, duty_cycleA);
    	Last_Throttle = throttle;
        /*
            USART1_Send('X');
            USART1_Send(':');
            USART1_Send(' ');
            Display_Axis(XTotal_Rotation);
            USART1_Send('Y');
            USART1_Send(':');
            USART1_Send(' ');
            Display_Axis(YTotal_Rotation);
            USART1_Send('z');
            USART1_Send(':');
            USART1_Send(' ');
            Display_Axis(ZTotal_Rotation);
            USART1_Send('\n');
            USART1_Send('\r');
 */

            USART1_Send('x');
        	USART1_Send_Int((int)(XTotal_Rotation));
        	//USART1_Send_Int((int)(pitch1*57));
            USART1_Send('-');
            USART1_Send('y');
        	USART1_Send_Int((int)(YTotal_Rotation));
        	//USART1_Send_Int((int)(roll1*57));
            USART1_Send('-');
            USART1_Send('z');
            //USART1_Send_Int((int)(alt));
        	//USART1_Send_Int((int)(AccBuffer2[2]/10));
            USART1_Send_Int((int)(heading));
            USART1_Send('-');
            USART1_Send('t');
            //USART1_Send_Int((int)(alt));
        	//USART1_Send_Int((int)(AccBuffer2[2]/10));
            USART1_Send_Int((int)(throttle-180));
            USART1_Send('-');



    }
}
