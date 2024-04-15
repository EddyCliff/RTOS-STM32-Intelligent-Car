#ifndef _INV_MPU_USER_H_
#define _INV_MPU_USER_H_

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "delay.h"
#include "usart.h"


u8 mpu_dmp_get_data(void);
u8 mpu_dmp_init(void);
	
#endif
