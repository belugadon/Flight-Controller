/**
  ******************************************************************************
  * @file    main.c 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    20-September-2012
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include <stdio.h>
#include <math.h>
//#include "GPS.h"
#include "stm32f3_discovery_l3gd20.h"
#include "stm32f3_discovery_lsm303dlhc.h"
#include "stm32f3_discovery.h "


#define ABS(x)         (x < 0) ? (-x) : x
#define RadToDeg                   (uint32_t)  57295

#define L3G_Sensitivity_250dps     (float)   114.285f         /*!< gyroscope sensitivity with 250 dps full scale [LSB/dps] */
#define L3G_Sensitivity_500dps     (float)    57.1429f        /*!< gyroscope sensitivity with 500 dps full scale [LSB/dps] */
#define L3G_Sensitivity_2000dps    (float)    14.285f	      /*!< gyroscope sensitivity with 2000 dps full scale [LSB/dps] */
#define PI                         (float)     3.14159265f

#define LSM_Acc_Sensitivity_2g     (float)     1.0f            /*!< accelerometer sensitivity with 2 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_4g     (float)     0.5f            /*!< accelerometer sensitivity with 4 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_8g     (float)     0.25f           /*!< accelerometer sensitivity with 8 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_16g    (float)     0.0834f         /*!< accelerometer sensitivity with 12 g full scale [LSB/mg] */
/* Private variables ---------------------------------------------------------*/
  RCC_ClocksTypeDef RCC_Clocks;
__IO uint32_t TimingDelay = 0;
__IO uint32_t UserButtonPressed = 0;

uint32_t calibration_value;
float MagBuffer[3] = {0.0f}, AccBuffer[3] = {0.0f}, Buff[3] = {0.0f};
int offset;
int IN_CH1 = 0;
int IN_CH2 = 0;
int IN_CH3 = 0;
int IN_CH4 = 0;
int RX_Watchdog = 0;
float pitch = 0;
float roll = 0;
float Initial_Heading[1] = {0.0f};
int IN_CH1_OFFSET = 0;
int IN_CH2_OFFSET = 0;
int IN_CH3_OFFSET = 0;
int IN_CH4_OFFSET = 0;

__IO uint8_t DataReady = 0;
__IO uint8_t PrevXferComplete = 1;
__IO uint32_t USBConnectTimeOut = 100;

float fNormAcc,fSinRoll,fCosRoll,fSinPitch,fCosPitch = 0.0f, RollAng = 0.0f, PitchAng = 0.0f;
float fTiltedX,fTiltedY = 0.0f;

uint8_t rx;
float Gyro_XOffset = 0;
float Gyro_YOffset = 0;
float Gyro_ZOffset = 0;

/**
  * @brief  Main program.
  * @param  None 
  * @retval None
  */

int main(void)
{

    /* SysTick end of count event each 10ms */
    RCC_GetClocksFreq(&RCC_Clocks);
    SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);
    RCC_Configuration();
    GPIO_Configuration();
    GPIO_Configuration2();
    Config_ADC();
    USART1_Configuration_Slow();

   
    GyroConfig();
    CompassConfig();
    init_pwm_gpio();
    init_pwm(300);
	PWMInput_Config();
	Set_Offset(&IN_CH3, &roll, &pitch, &IN_CH4);

	Calibrate_RX_Inputs();
	Calculate_Gyro_Drift();
	Initialize_Position();
	schedule_PI_interrupts();
	while(1)
	{

		Get_Control_Channels();
		Set_Offset(&IN_CH3, &roll, &pitch, &IN_CH4);
	    Calculate_Position();
	}
}

void Calculate_Heading_Bias(float* pfData)
{
	float sum_of_headings = 0.0;
	float head_reading[1] = {0.0f};
	int i = 0;
	for(i=0;i<10;i=i+1){
	get_heading(head_reading);
	sum_of_headings = sum_of_headings + head_reading[0];
	}
	pfData[0] = sum_of_headings/10;

}
void PWMInput_Config()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM15, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_2);
	GPIO_ResetBits(GPIOD,GPIO_Pin_12);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_2);
	GPIO_ResetBits(GPIOC,GPIO_Pin_6);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_2);
	GPIO_ResetBits(GPIOA,GPIO_Pin_15);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource9, GPIO_AF_3);
	GPIO_ResetBits(GPIOF,GPIO_Pin_9);

	TIM_TimeBaseInitTypeDef timerInitStructure;
	timerInitStructure.TIM_Prescaler = 0x0000;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = 0xFFFF;
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	timerInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &timerInitStructure);

	timerInitStructure.TIM_Prescaler = 0x0000;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = 0xFFFF;
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	timerInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &timerInitStructure);

	timerInitStructure.TIM_Prescaler = 0x0000;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = 0xFFFF;
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	timerInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM8, &timerInitStructure);

	timerInitStructure.TIM_Prescaler = 0x0000;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = 0xFFFF;
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	timerInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM15, &timerInitStructure);

    TIM_ICInitTypeDef inputCaptureInitStructure;
    TIM_ICStructInit(&inputCaptureInitStructure);
    inputCaptureInitStructure.TIM_Channel = TIM_Channel_1;
    inputCaptureInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    inputCaptureInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    inputCaptureInitStructure.TIM_ICFilter = 0x01;
    TIM_PWMIConfig(TIM4, &inputCaptureInitStructure);

    TIM_ICStructInit(&inputCaptureInitStructure);
    inputCaptureInitStructure.TIM_Channel = TIM_Channel_1;
    inputCaptureInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    inputCaptureInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    inputCaptureInitStructure.TIM_ICFilter = 0x01;
    TIM_PWMIConfig(TIM3, &inputCaptureInitStructure);

    TIM_ICStructInit(&inputCaptureInitStructure);
    inputCaptureInitStructure.TIM_Channel = TIM_Channel_1;
    inputCaptureInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    inputCaptureInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    inputCaptureInitStructure.TIM_ICFilter = 0x01;
    TIM_PWMIConfig(TIM8, &inputCaptureInitStructure);

    TIM_ICStructInit(&inputCaptureInitStructure);
    inputCaptureInitStructure.TIM_Channel = TIM_Channel_1;
    inputCaptureInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    inputCaptureInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    inputCaptureInitStructure.TIM_ICFilter = 0x01;
    TIM_PWMIConfig(TIM15, &inputCaptureInitStructure);

    TIM_SelectInputTrigger(TIM4, TIM_TS_TI1FP1);
    TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset);
    TIM_SelectMasterSlaveMode(TIM4,TIM_MasterSlaveMode_Enable);

    TIM_SelectInputTrigger(TIM3, TIM_TS_TI1FP1);
    TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);
    TIM_SelectMasterSlaveMode(TIM3,TIM_MasterSlaveMode_Enable);

    TIM_SelectInputTrigger(TIM8, TIM_TS_TI1FP1);
    TIM_SelectSlaveMode(TIM8, TIM_SlaveMode_Reset);
    TIM_SelectMasterSlaveMode(TIM8,TIM_MasterSlaveMode_Enable);

    TIM_SelectInputTrigger(TIM15, TIM_TS_TI1FP1);
    TIM_SelectSlaveMode(TIM15, TIM_SlaveMode_Reset);
    TIM_SelectMasterSlaveMode(TIM15,TIM_MasterSlaveMode_Enable);

    TIM_ClearITPendingBit(TIM4, TIM_IT_Update|TIM_IT_CC3|TIM_IT_CC4);
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update|TIM_IT_CC3|TIM_IT_CC4);
    TIM_ClearITPendingBit(TIM8, TIM_IT_Update|TIM_IT_CC3|TIM_IT_CC4);
    TIM_ClearITPendingBit(TIM15, TIM_IT_Update|TIM_IT_CC3|TIM_IT_CC4);

    TIM_ITConfig(TIM4, TIM_IT_CC1|TIM_IT_CC2, ENABLE);
    TIM_ITConfig(TIM3, TIM_IT_CC1|TIM_IT_CC2, ENABLE);
    TIM_ITConfig(TIM8, TIM_IT_CC1|TIM_IT_CC2, ENABLE);
    TIM_ITConfig(TIM15, TIM_IT_CC1|TIM_IT_CC2, ENABLE);

    TIM_Cmd(TIM4, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
    TIM_Cmd(TIM8, ENABLE);
    TIM_Cmd(TIM15, ENABLE);
}

void Calibrate_RX_Inputs()
{
	int Sample = 0;
	int Sum = 0;
	int i = 0;
	for(i=0;i<100;i++){
	while((Sample == 0) || (Sample >= 15000)){
		if (TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET)
		{
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);
		Sample = TIM4->CCR2;
		Sum = Sum + Sample;
		}
	}
	Sample = 0;
	}
	IN_CH1_OFFSET = Sum/100;
	Sum = 0;
	for(i=0;i<100;i++){
	while((Sample == 0) || (Sample >= 15000))
	{
		if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)
		{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
		Sample = TIM3->CCR2;
		Sum = Sum + Sample;
		}
	}
	Sample = 0;
	}
	IN_CH2_OFFSET = Sum/100;
	Sum = 0;
	for(i=0;i<10;i++){
	while((Sample == 0) || (Sample >= 20000))
	{
		if (TIM_GetITStatus(TIM15, TIM_IT_CC2) != RESET)
		{
		TIM_ClearITPendingBit(TIM15, TIM_IT_CC2);
		Sample = TIM15->CCR2;
		Sum = Sum + Sample;
		}
	}
	Sample = 0;
	}
	IN_CH4_OFFSET = Sum/10;
	Sum = 0;
}
void Get_Control_Channels()
{
	if (TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET)
	{
	TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);
	IN_CH1 = TIM4->CCR2 - IN_CH1_OFFSET;
	roll = IN_CH1;//(IN_CH1/40);
	}
	if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)
	{
	TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
	IN_CH2 = TIM3->CCR2 - IN_CH2_OFFSET;
	pitch = (0 - IN_CH2);//(0 - (IN_CH2/40));
	}
	if (TIM_GetITStatus(TIM8, TIM_IT_CC2) != RESET)
	{
	TIM_ClearITPendingBit(TIM8, TIM_IT_CC2);
	IN_CH3 = TIM8->CCR2;
	IN_CH3 = (IN_CH3 - 9161);
	IN_CH3 = IN_CH3 * 1.75;
	RX_Watchdog = 0;
	}
	else
	{
	RX_Watchdog = RX_Watchdog + 1;
	}
	if (TIM_GetITStatus(TIM15, TIM_IT_CC2) != RESET)
	{
	TIM_ClearITPendingBit(TIM15, TIM_IT_CC2);
	IN_CH4 = TIM15->CCR2 - IN_CH4_OFFSET;
	//IN_CH4 = IN_CH4 / 25;
	}
	if(RX_Watchdog > 100)
	{
		roll = 0;
		pitch = 0;
		IN_CH3 = 0;
		IN_CH4 = 0;
	}
}

void get_heading(float* pfData)
{
	uint8_t i;
	DataReady = 0x00;
    CompassConfig();

    /* Wait for data ready */
    while(DataReady !=0x05)
    {}
    DataReady = 0x00;
    CompassReadMag(MagBuffer);
    CompassReadAcc(AccBuffer);

    for(i=0;i<3;i++)
        AccBuffer[i] /= 100.0f;

      fNormAcc = sqrt((AccBuffer[0]*AccBuffer[0])+(AccBuffer[1]*AccBuffer[1])+(AccBuffer[2]*AccBuffer[2]));

      fSinRoll = -AccBuffer[1]/fNormAcc;
      fCosRoll = sqrt(1.0-(fSinRoll * fSinRoll));
      fSinPitch = AccBuffer[0]/fNormAcc;
      fCosPitch = sqrt(1.0-(fSinPitch * fSinPitch));

	fTiltedX = MagBuffer[0]*fCosPitch+MagBuffer[2]*fSinPitch;
    fTiltedY = MagBuffer[0]*fSinRoll*fSinPitch+MagBuffer[1]*fCosRoll-MagBuffer[1]*fSinRoll*fCosPitch;
    pfData[0] = (float) ((atan2f((float)fTiltedY,(float)fTiltedX))*RadToDeg);//*180)/PI;
    //pfData[0] = pfData[0] + 180000.0;
    ///pfData[0] = pfData[0] / 10;

    //Display_Heading(HeadingValue);
}

/**
  * @brief  Configure the Mems to gyroscope application.
  * @param  None
  * @retval None
  */
void GyroConfig(void)
{
  L3GD20_InitTypeDef L3GD20_InitStructure;
  L3GD20_FilterConfigTypeDef L3GD20_FilterStructure;
  
  /* Configure Mems L3GD20 */
  L3GD20_InitStructure.Power_Mode = L3GD20_MODE_ACTIVE;
  L3GD20_InitStructure.Output_DataRate = L3GD20_OUTPUT_DATARATE_1;
  L3GD20_InitStructure.Axes_Enable = L3GD20_AXES_ENABLE;
  L3GD20_InitStructure.Band_Width = L3GD20_BANDWIDTH_4;
  L3GD20_InitStructure.BlockData_Update = L3GD20_BlockDataUpdate_Continous;
  L3GD20_InitStructure.Endianness = L3GD20_BLE_LSB;
  L3GD20_InitStructure.Full_Scale = L3GD20_FULLSCALE_250;
  L3GD20_Init(&L3GD20_InitStructure);
   
  L3GD20_FilterStructure.HighPassFilter_Mode_Selection =L3GD20_HPM_NORMAL_MODE_RES;
  L3GD20_FilterStructure.HighPassFilter_CutOff_Frequency = L3GD20_HPFCF_0;
  L3GD20_FilterConfig(&L3GD20_FilterStructure) ;
  
  L3GD20_FilterCmd(L3GD20_HIGHPASSFILTER_ENABLE);
}
/**
  * @brief  Calculate the angular Data rate Gyroscope.
  * @param  pfData : Data out pointer
  * @retval None
  */
void GyroReadAngRate (float* pfData)
{
  uint8_t tmpbuffer[6] ={0};
  int16_t RawData[3] = {0};
  uint8_t tmpreg = 0;
  float sensitivity = 0;
  int i =0;

  L3GD20_Read(&tmpreg,L3GD20_CTRL_REG4_ADDR,1);
  
  L3GD20_Read(tmpbuffer,L3GD20_OUT_X_L_ADDR,6);
  
  /* check in the control register 4 the data alignment (Big Endian or Little Endian)*/
  if(!(tmpreg & 0x40))
  {
    for(i=0; i<3; i++)
    {
      RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i+1] << 8) + tmpbuffer[2*i]);
    }
  }
  else
  {
    for(i=0; i<3; i++)
    {
      RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i] << 8) + tmpbuffer[2*i+1]);
    }
  }
  
  /* Switch the sensitivity value set in the CRTL4 */
  switch(tmpreg & 0x30)
  {
  case 0x00:
    sensitivity=L3G_Sensitivity_250dps;
    break;
    
  case 0x10:
    sensitivity=L3G_Sensitivity_500dps;
    break;
    
  case 0x20:
    sensitivity=L3G_Sensitivity_2000dps;
    break;
  }
  /* divide by sensitivity */
  for(i=0; i<3; i++)
  {
    pfData[i]=(float)RawData[i]/sensitivity;
  }
  pfData[0] = pfData[0] - Gyro_XOffset;
  pfData[1] = pfData[1] - Gyro_YOffset;
  pfData[2] = pfData[2] - Gyro_ZOffset;
}

void Calculate_Gyro_Drift()
{
/*
 * This function repeatedly samples the gyro's output during an assumed period of inactivity.
 * these samples are summed and divided by the total number of samples to give the average
 * drift of the gyro. The value calculated is subtracted from the observed gyro output
 */
	uint8_t i;
	float X_SUM = 0;
	float Y_SUM = 0;
	float Z_SUM = 0;
	Gyro_XOffset = 0;
	Gyro_YOffset = 0;
	Gyro_ZOffset = 0;

	for (i=0;i<10;i++){ GyroReadAngRate(Buff); }

	for (i=0;i<50;i++)
	{
		GyroReadAngRate(Buff);
		X_SUM = X_SUM + Buff[0];
		Y_SUM = Y_SUM + Buff[1];
		Z_SUM = Z_SUM + Buff[2];
	}
	Gyro_XOffset = X_SUM/51;
	Gyro_YOffset = Y_SUM/51;
	Gyro_ZOffset = Z_SUM/51;
}

/**
  * @brief  Configure the Mems to compass application.
  * @param  None
  * @retval None
  */
void CompassConfig(void)
{
  LSM303DLHCMag_InitTypeDef LSM303DLHC_InitStructure;
  LSM303DLHCAcc_InitTypeDef LSM303DLHCAcc_InitStructure;
  LSM303DLHCAcc_FilterConfigTypeDef LSM303DLHCFilter_InitStructure;

  /* Configure MEMS magnetometer main parameters: temp, working mode, full Scale and Data rate */
  LSM303DLHC_InitStructure.Temperature_Sensor = LSM303DLHC_TEMPSENSOR_DISABLE;
  LSM303DLHC_InitStructure.MagOutput_DataRate =LSM303DLHC_ODR_30_HZ ;
  LSM303DLHC_InitStructure.MagFull_Scale = LSM303DLHC_FS_8_1_GA;
  LSM303DLHC_InitStructure.Working_Mode = LSM303DLHC_CONTINUOS_CONVERSION;
  LSM303DLHC_MagInit(&LSM303DLHC_InitStructure);

   /* Fill the accelerometer structure */
  LSM303DLHCAcc_InitStructure.Power_Mode = LSM303DLHC_NORMAL_MODE;
  LSM303DLHCAcc_InitStructure.AccOutput_DataRate = LSM303DLHC_ODR_50_HZ;
  LSM303DLHCAcc_InitStructure.Axes_Enable= LSM303DLHC_AXES_ENABLE;
  LSM303DLHCAcc_InitStructure.AccFull_Scale = LSM303DLHC_FULLSCALE_2G;
  LSM303DLHCAcc_InitStructure.BlockData_Update = LSM303DLHC_BlockUpdate_Continous;
  LSM303DLHCAcc_InitStructure.Endianness=LSM303DLHC_BLE_LSB;
  LSM303DLHCAcc_InitStructure.High_Resolution=LSM303DLHC_HR_ENABLE;
  /* Configure the accelerometer main parameters */
  LSM303DLHC_AccInit(&LSM303DLHCAcc_InitStructure);

  /* Fill the accelerometer LPF structure */
  LSM303DLHCFilter_InitStructure.HighPassFilter_Mode_Selection =LSM303DLHC_HPM_NORMAL_MODE;
  LSM303DLHCFilter_InitStructure.HighPassFilter_CutOff_Frequency = LSM303DLHC_HPFCF_16;
  LSM303DLHCFilter_InitStructure.HighPassFilter_AOI1 = LSM303DLHC_HPF_AOI1_DISABLE;
  LSM303DLHCFilter_InitStructure.HighPassFilter_AOI2 = LSM303DLHC_HPF_AOI2_DISABLE;

  /* Configure the accelerometer LPF main parameters */
  LSM303DLHC_AccFilterConfig(&LSM303DLHCFilter_InitStructure);
}
/**
* @brief Read LSM303DLHC output register, and calculate the acceleration ACC=(1/SENSITIVITY)* (out_h*256+out_l)/16 (12 bit rappresentation)
* @param pnData: pointer to float buffer where to store data
* @retval None
*/
void CompassReadAcc(float* pfData)
{
  int16_t pnRawData[3];
  uint8_t ctrlx[2];
  uint8_t buffer[6], cDivider;
  uint8_t i = 0;
  float LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_2g;

  /* Read the register content */
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, ctrlx,2);
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_X_L_A, buffer, 6);

  if(ctrlx[1]&0x40)
    cDivider=64;
  else
    cDivider=16;

  /* check in the control register4 the data alignment*/
  if(!(ctrlx[0] & 0x40) || (ctrlx[1] & 0x40)) /* Little Endian Mode or FIFO mode */
  {
    for(i=0; i<3; i++)
    {
      pnRawData[i]=((int16_t)((uint16_t)buffer[2*i+1] << 8) + buffer[2*i])/cDivider;
    }
  }
  else /* Big Endian Mode */
  {
    for(i=0; i<3; i++)
      pnRawData[i]=((int16_t)((uint16_t)buffer[2*i] << 8) + buffer[2*i+1])/cDivider;
  }
  /* Read the register content */
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, ctrlx,2);


  if(ctrlx[1]&0x40)
  {
    /* FIFO mode */
    LSM_Acc_Sensitivity = 0.25;
  }
  else
  {
    /* normal mode */
    /* switch the sensitivity value set in the CRTL4*/
    switch(ctrlx[0] & 0x30)
    {
    case LSM303DLHC_FULLSCALE_2G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_2g;
      break;
    case LSM303DLHC_FULLSCALE_4G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_4g;
      break;
    case LSM303DLHC_FULLSCALE_8G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_8g;
      break;
    case LSM303DLHC_FULLSCALE_16G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_16g;
      break;
    }
  }

  /* Obtain the mg value for the three axis */
  for(i=0; i<3; i++)
  {
    pfData[i]=(float)pnRawData[i]/LSM_Acc_Sensitivity;
  }

}
/**
  * @brief  calculate the magnetic field Magn.
* @param  pfData: pointer to the data out
  * @retval None
  */
void CompassReadMag (float* pfData)
{
  static uint8_t buffer[6] = {0};
  uint8_t CTRLB = 0;
  uint16_t Magn_Sensitivity_XY = 0, Magn_Sensitivity_Z = 0;
  uint8_t i =0;
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_CRB_REG_M, &CTRLB, 1);

  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_H_M, buffer, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_L_M, buffer+1, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_H_M, buffer+2, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_L_M, buffer+3, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_H_M, buffer+4, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_L_M, buffer+5, 1);
  /* Switch the sensitivity set in the CRTLB*/
  switch(CTRLB & 0xE0)
  {
  case LSM303DLHC_FS_1_3_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_1_3Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_1_3Ga;
    break;
  case LSM303DLHC_FS_1_9_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_1_9Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_1_9Ga;
    break;
  case LSM303DLHC_FS_2_5_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_2_5Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_2_5Ga;
    break;
  case LSM303DLHC_FS_4_0_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_4Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_4Ga;
    break;
  case LSM303DLHC_FS_4_7_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_4_7Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_4_7Ga;
    break;
  case LSM303DLHC_FS_5_6_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_5_6Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_5_6Ga;
    break;
  case LSM303DLHC_FS_8_1_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_8_1Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_8_1Ga;
    break;
  }

  for(i=0; i<2; i++)
  {
    pfData[i]=(float)((int16_t)(((uint16_t)buffer[2*i] << 8) + buffer[2*i+1])*1000)/Magn_Sensitivity_XY;
  }
  pfData[2]=(float)((int16_t)(((uint16_t)buffer[4] << 8) + buffer[5])*1000)/Magn_Sensitivity_Z;
}

void Config_ADC(uint16_t GPIO_PIN, uint16_t ADC_Channel)
{
	uint32_t timedelay = 100;
    ADC_InitTypeDef       ADC_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    GPIO_InitTypeDef      GPIO_InitStructure;

    // Configure the ADC clock
    RCC_ADCCLKConfig( RCC_ADC12PLLCLK_Div2 );

    // Enable ADC1 clock
    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_ADC12, ENABLE );

    // ADC Channel configuration
    // GPIOC Periph clock enable
    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOC, ENABLE );

    // Configure ADC Channel7 as analog input
    GPIO_InitStructure.GPIO_Pin = GPIO_PIN ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init( GPIOC, &GPIO_InitStructure );

    ADC_StructInit( &ADC_InitStructure );

    // Calibration procedure
    ADC_VoltageRegulatorCmd( ADC1, ENABLE );

    // Insert delay equal to 10 µs
    //TaskDelay(5);
    Timing_Delay(timedelay);

    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Clock = ADC_Clock_SynClkModeDiv4; //ADC_Clock_AsynClkMode;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_OneShot;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;
    ADC_CommonInit( ADC1, &ADC_CommonInitStructure );

    ADC_SelectCalibrationMode( ADC1, ADC_CalibrationMode_Single );
    ADC_StartCalibration( ADC1 );

    while ( ADC_GetCalibrationStatus( ADC1 ) != RESET );
    calibration_value = ADC_GetCalibrationValue( ADC1 );

    ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Disable;
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;
    ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;
    ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;
    ADC_InitStructure.ADC_NbrOfRegChannel = 1;
    ADC_Init( ADC1, &ADC_InitStructure );

    // ADC1 regular channel configuration
    ADC_RegularChannelConfig( ADC1, ADC_Channel, 1, ADC_SampleTime_7Cycles5 );

    // Enable ADC1
    ADC_Cmd( ADC1, ENABLE );

    // wait for ADRDY
    while( !ADC_GetFlagStatus( ADC1, ADC_FLAG_RDY ) );

    // Start ADC1 Software Conversion
    ADC_StartConversion( ADC1 );
}
uint16_t Get_Voltage()
{
	 	uint16_t  ADC1ConvertedValue = 0;

	    // Test EOC flag
	    while ( ADC_GetFlagStatus( ADC1, ADC_FLAG_EOC ) == RESET );

	    // Get ADC1 converted data
	    ADC1ConvertedValue = ADC_GetConversionValue( ADC1 );

	    // Compute the voltage (3.3V @ 12bit resolution)
	    //ADC1ConvertedValue = ( ADC1ConvertedValue * 3300 ) / 0xFFF;
	    //ADC1ConvertedValue = ( ADC1ConvertedValue * 3000 ) / 0xFFF;

	    ADC_StopConversion(ADC1);
	    //ADC_DisableCmd(ADC1);
	    ADC_DeInit(ADC1);

	    return ADC1ConvertedValue;
}
uint32_t Get_Baro()
{
	__IO uint32_t Pressure = 0;
	__IO uint32_t Voltage = 0;
	Config_ADC(GPIO_Pin_0, ADC_Channel_6);
	Voltage = Get_Voltage();

	//Voltage = Voltage * 10000;
	//Pressure = Voltage / 0.036278;//0.68*Vdd*a

	//Pressure = Pressure + 306157000;//-a/b

	return Voltage;
}
/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
	  if (TimingDelay != 0x00)
	  {
		  TimingDelay--;
	  }
}
void Timing_Delay(uint32_t Delay)
{
  if (Delay != 0x00)
  { 
    Delay--;
  }
}
/**
  * @brief  Basic management of the timeout situation.
  * @param  None.
  * @retval None.
  */
uint32_t L3GD20_TIMEOUT_UserCallback(void)
{
  return 0;
}
/**
  * @brief  Basic management of the timeout situation.
  * @param  None.
  * @retval None.
  */
uint32_t LSM303DLHC_TIMEOUT_UserCallback(void)
{
  return 0;
}
void bt_uart(){
	int i = 0;
	char ATZ[] = {'A','T','Z','\r'};
	char uart[] = {'A','T','+','U','A','R','T','C','O','N','F','I','G',',','1','9','2','0','0',',','N',',','1',',','0','\r'};
	for (i=0;i<4;i=i+1){
		USART1_Send(ATZ[i]);
	}
	for (i=0;i<26;i=i+1){
			USART1_Send(uart[i]);
	}
}
void bluetooth_setup(){
	int i = 0;
	char mode[] = {'A','T','+','B','T','M','O','D','E',',','3','\r'};
	char ATZ[] = {'A','T','Z','\r'};
	for (i=0;i<12;i=i+1){
		USART1_Send(mode[i]);
	}
	for (i=0;i<4;i=i+1){
		USART1_Send(ATZ[i]);
	}


}
void RCC_Configuration(void)
{
  /// Enable GPIO clock
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  // Enable USART clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
}

void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // Connect PA9 to USART1_Tx
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_7);

  // Connect PA10 to USART1_Rx
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_7);

  // Configure USART Tx as alternate function push-pull
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // Configure USART Rx as alternate function push-pull
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}
void GPIO_Configuration2(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // Connect PD5 to USART1_Tx
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_7);

  // Connect PD6 to USART1_Rx
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_7);

  // Configure USART Tx as alternate function push-pull
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // Configure USART Rx as alternate function push-pull
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void USART1_Configuration_Slow(void)
{
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  // USART configuration
  USART_Init(USART1, &USART_InitStructure);
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
  USART_ITConfig(USART1, USART_IT_TC, DISABLE);

  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  // Enable USART
  USART_Cmd(USART1, ENABLE);
}
void USART1_Configuration_Fast(void)
{
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  USART_InitStructure.USART_BaudRate = 19200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  // USART configuration
  USART_Init(USART1, &USART_InitStructure);
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);


  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  // Enable USART
  USART_Cmd(USART1, ENABLE);
}
void USART3_Configuration(void)
{
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  // USART configuration
  USART_Init(USART3, &USART_InitStructure);
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  // Enable USART
  USART_Cmd(USART3, ENABLE);
}
void USART1_Send(char character)
{
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); // Wait for Empty

    USART_SendData(USART1, character);
}



void Display_Pulse_Width(uint16_t value)
{
	char dig, i;
	char message[4];
	uint32_t a = 1;
	for(i=1; i <= 5; i=i+1)
	{
		dig = value % 10;
		value = value / 10;
		message[i]=dig+48;
	}
	for(i=5; i > 0; i=i-1)
	{
		USART1_Send(message[i]);
	}
	USART1_Send(' ');
	USART1_Send(' ');

}

void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET){
	//while((USART1->ISR & USART_FLAG_RXNE) == (uint16_t)RESET);
	rx = USART_ReceiveData(USART1);
	}
	//USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
	//USART_ITConfig(USART1, USART_IT_TC, DISABLE);
}
void USART3_IRQHandler(void)
{
	while((USART3->ISR & USART_FLAG_RXNE) == (uint16_t)RESET);
	rx = USART_ReceiveData(USART3);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
