#include "KalmanFilter.h"

float XQ_angle  =  0.001;
float XQ_bias   =  0.003;
float XR_measured  =  0.03;
float YQ_angle  =  0.001;
float YQ_bias   =  0.003;
float YR_measured  =  0.03;

float X_angle = 0;
float X_bias = 0;
float Y_angle = 0;
float Y_bias = 0;

float XP_00 = 0, XP_01 = 0, XP_10 = 0, XP_11 = 0;
float Xdt, Xy, XS;
float XK_0, XK_1;
float YP_00 = 0, YP_01 = 0, YP_10 = 0, YP_11 = 0;
float Ydt, Yy, YS;
float YK_0, YK_1;


float kalmanFilterX(float newAngle, float newRate,int dt){

	X_angle += dt * (newRate - X_bias);
    XP_00 +=  dt * (dt * XP_11 - XP_10 - XP_01) + XQ_angle;
    XP_01 +=  - dt * XP_11;
    XP_10 +=  - dt * XP_11;
    XP_11 +=  + XQ_bias;

    XS = XP_00 + XR_measured;
    XK_0 = XP_00 / XS;
    XK_1 = XP_10 / XS;

    Xy = newAngle - X_angle;
    X_angle +=  XK_0 * Xy;
    X_bias  +=  XK_1 * Xy;

    XP_00 -= XK_0 * XP_00;
    XP_01 -= XK_0 * XP_01;
    XP_10 -= XK_1 * XP_00;
    XP_11 -= XK_1 * XP_01;

    return X_angle;
}
float kalmanFilterY(float newAngle, float newRate,int dt){

	Y_angle += dt * (newRate - Y_bias);
    YP_00 +=  dt * (dt * YP_11 - YP_10 - YP_01) + YQ_angle;
    YP_01 +=  - dt * YP_11;
    YP_10 +=  - dt * YP_11;
    YP_11 +=  + YQ_bias;

    YS = YP_00 + YR_measured;
    YK_0 = YP_00 / YS;
    YK_1 = YP_10 / YS;

    Yy = newAngle - Y_angle;
    Y_angle +=  YK_0 * Yy;
    Y_bias  +=  YK_1 * Yy;

    YP_00 -= YK_0 * YP_00;
    YP_01 -= YK_0 * YP_01;
    YP_10 -= YK_1 * YP_00;
    YP_11 -= YK_1 * YP_01;

    return Y_angle;
}
