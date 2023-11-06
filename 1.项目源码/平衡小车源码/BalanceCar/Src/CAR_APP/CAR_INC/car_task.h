#ifndef __CAR_TASK_H
#define __CAR_TASK_H
#include "stm32f4xx_hal.h"


extern int  FS_MODE  ;                      //0、遥控模式   1、蔽障模式  2、巡线模式 
extern int  Balance_Pwm,Velocity_Pwm,Turn_Pwm;        //PID计算的PWM值
extern int  Motor1, Motor2;                  //左右电机PWM值
extern int  Encoder_left, Encoder_right;     //检测速度
extern float Movement ;                       //速度调节  
extern int  Contrl_Turn ;                     //转向调节变量
extern int  Distence ;                       //小车和前方障碍物之间的距离
extern uint8_t   power;                       //定义电池电量



struct mpu6050_data{
	
		short acc_x;
		short acc_y;
		short acc_z;
		
		short gyro_x;
		short gyro_y;
		short gyro_z;
	
		float pitch;    //俯仰角
	  float roll;     //翻滚角
	  float yaw;      //偏航角
};

extern struct mpu6050_data OutMpu;



struct tCCD
{
	uint16_t middle;      //中间位置值
	uint16_t threshold;   //像素ad阈值
	uint16_t left;        //左跳变的位置
	uint16_t right;       //右跳变的位置
};

extern struct tCCD  CCD;




extern  int  Distence ;                       //小车和前方障碍物之间的距离


void Car_Task_200HZ(void);
void Car_Task_100HZ(void);
void Car_Task_5HZ(void);

void  HC_SRC04_Start(void);



#endif
