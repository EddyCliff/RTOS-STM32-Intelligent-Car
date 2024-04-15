#ifndef __OLED_SHOW_H
#define __OLED_SHOW_H
#include "sys.h"



#define KEY0 		HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3) 
#define KEY1 		HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4)	

#define KEY2 		HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_4) 
#define KEY3 		HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_6)	
#define KEY4 		HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_7) 

#define KEY_LEFT 			1
#define KEY_UP 				2
#define KEY_DOWN  		3
#define KEY_RIGHT 		5
#define KEY_CENTER 		4


#define LED_B		PBout(8) //RGBÀ¶µÆ
#define LED_G		PBout(9) //RGBÂÌµÆ
#define LED_R		PEout(0) //RGBºìµÆ
#define BUZZ		PBout(7) //·äÃùÆ÷

struct _PARMETER
{
	char	Upright_PD[20];
	char	Speed_PI[20];
	char	Turn_PD[20];
	char	Mechanical_Zero[20];
};




void oled_Show(void);
void Interface(void);

#endif


