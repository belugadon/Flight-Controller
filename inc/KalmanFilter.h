#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f30x.h"
#include "stm32f3_discovery.h"




float kalmanFilterY(float newAngle, float newRate,int dt);
float kalmanFilterX(float newAngle, float newRate,int dt);


#endif
