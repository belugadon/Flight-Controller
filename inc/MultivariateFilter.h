#ifndef __MULTIVARIATEFILTER_H
#define __MULTIVARIATEFILTER_H

#ifdef __cplusplus
 extern "C" {
#endif


#include "stm32f30x.h"
#include "stm32f3_discovery.h"

typedef struct
{
	float z;
	float dt;
	float angle;
	float previous_angle;
	float velocity;
	float Q_angle;
	float Q_bias;
	float R_measured;
	float P_00;
	float P_01;
	float P_10;
	float P_11;
} KalmanFilterTypeDef;

float kalmanFilter_Init(KalmanFilterTypeDef* filter);
float kalmanFilter(KalmanFilterTypeDef* filter);



#endif
