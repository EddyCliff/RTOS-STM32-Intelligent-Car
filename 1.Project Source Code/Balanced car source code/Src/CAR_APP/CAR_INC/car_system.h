#ifndef __CAR_SYSTEM_H
#define __CAR_SYSTEM_H

#include <stdint.h>
#include <stdio.h>
#include "delay.h"




void  Led_Contrl(uint8_t cmd);
void HC_SR04_Check(void);
int Get_battery_volt(void) ;
void Task_State(void);


#endif

