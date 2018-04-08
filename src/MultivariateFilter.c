#include "MultivariateFilter.h"



float kalmanFilter_Init(KalmanFilterTypeDef* filter){
	filter->angle = 0;
	filter->previous_angle = 0;
	filter->velocity = 0;
	filter->Q_angle  = 0.01;
	filter->Q_bias   =  0.003;
	filter->R_measured  =  0.03;
	filter->P_00 = 0;
	filter->P_01 = 0;
	filter->P_10 = 0;
	filter->P_11 = 0;
}
float kalmanFilter(KalmanFilterTypeDef* filter){

	float y, S = 0;
	float K_0, K_1;
	/*Position prediction*/
	filter->angle =  filter->angle + (filter->dt * (filter->velocity));
	//x_velocity = (0 * X_angle) + (1 * x_velocity);
	//X_angle = X_angle + u;

	/*Variance prediction*/
	filter->P_00 = filter->P_00 + filter->dt * (filter->dt * filter->P_11 + filter->P_10 + filter->P_01) + filter->Q_angle;
	filter->P_01 = filter->P_01+ (filter->dt * filter->P_11);
	filter->P_10 = filter->P_10 + (filter->dt * filter->P_11);
	filter->P_11 = filter->P_11 + filter->Q_bias;

    /*Kalman gain calculation*/
    S = filter->P_00 + filter->R_measured;
    K_0 = filter->P_00 / S;
    K_1 = filter->P_10 / S;

    /*Residual*/
    y = filter->z - filter->angle;

    if((y < 60) & (y > (0-60))){
    /*Posterior position calculation*/
    filter->angle = filter->angle + K_0 * y;
    //X_bias  = X_bias  + K_1 * y;

    filter->velocity = filter->angle - filter->previous_angle;
    filter->previous_angle = filter->angle;

    /*Posterior variance calculation*/
    filter->P_00 = filter->P_00 - (K_0 * filter->P_00);
    filter->P_01 = filter->P_01 - (K_0 * filter->P_01);
    filter->P_10 = filter->P_10 - (K_1 * filter->P_00);
    filter->P_11 = filter->P_11 - (K_1 * filter->P_01);
    }

    return filter->angle;
}

float Calculate_GyroGain(float Gyro, float Accel, float max_error)
{
	float gain = 0;
	gain = sqrt((Gyro - Accel)*(Gyro - Accel))/max_error;
	if (gain > 1){gain=1;}
	return gain;
}
