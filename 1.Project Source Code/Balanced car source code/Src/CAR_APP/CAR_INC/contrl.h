#ifndef _CONTRIL_H_
#define _CONTRIL_H_

#include "sys.h"




//机械0点
#define Mechanical_balance 0

#define AIN1(PinState)    HAL_GPIO_WritePin( GPIOE, GPIO_PIN_13, (GPIO_PinState)PinState)
#define AIN2(PinState)    HAL_GPIO_WritePin( GPIOE, GPIO_PIN_15, (GPIO_PinState)PinState)

#define BIN1(PinState)    HAL_GPIO_WritePin( GPIOC, GPIO_PIN_3, (GPIO_PinState)PinState)
#define BIN2(PinState)    HAL_GPIO_WritePin( GPIOA, GPIO_PIN_3, (GPIO_PinState)PinState)

#define PWMA   TIM4->CCR1 
#define PWMB   TIM5->CCR3



extern volatile int Encoder_Left,Encoder_Right;		      //编码器左右速度值

struct pid_arg{
	
	float Balance_Kp;
	float Balance_Ki;
	float Balance_Kd;
	
	float Velocity_Kp;
	float Velocity_Ki;
	float Velocity_Kd;
	
	float  Turn_Kp;
	float  Turn_Ki;
	float  Turn_Kd;

};
extern struct pid_arg PID;

int Read_Encoder(u8 TIMX);
int	Vertical_Ring_PD(float Angle,float Gyro);
int Vertical_speed_PI(int encoder_left,int encoder_right,float Angle,float Movement );
int Vertical_turn_PD(u8 CCD,short yaw);


void PWM_Limiting(int *motor1,int *motor2);
u8 Turn_off(const float Angle);
void Set_PWM(int motor1,int motor2);


#endif
