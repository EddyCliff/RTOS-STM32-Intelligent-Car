#ifndef __CAR_TASK_H
#define __CAR_TASK_H
#include "stm32f4xx_hal.h"


extern int  FS_MODE  ;                      //0��ң��ģʽ   1������ģʽ  2��Ѳ��ģʽ 
extern int  Balance_Pwm,Velocity_Pwm,Turn_Pwm;        //PID�����PWMֵ
extern int  Motor1, Motor2;                  //���ҵ��PWMֵ
extern int  Encoder_left, Encoder_right;     //����ٶ�
extern float Movement ;                       //�ٶȵ���  
extern int  Contrl_Turn ;                     //ת����ڱ���
extern int  Distence ;                       //С����ǰ���ϰ���֮��ľ���
extern uint8_t   power;                       //�����ص���



struct mpu6050_data{
	
		short acc_x;
		short acc_y;
		short acc_z;
		
		short gyro_x;
		short gyro_y;
		short gyro_z;
	
		float pitch;    //������
	  float roll;     //������
	  float yaw;      //ƫ����
};

extern struct mpu6050_data OutMpu;



struct tCCD
{
	uint16_t middle;      //�м�λ��ֵ
	uint16_t threshold;   //����ad��ֵ
	uint16_t left;        //�������λ��
	uint16_t right;       //�������λ��
};

extern struct tCCD  CCD;




extern  int  Distence ;                       //С����ǰ���ϰ���֮��ľ���


void Car_Task_200HZ(void);
void Car_Task_100HZ(void);
void Car_Task_5HZ(void);

void  HC_SRC04_Start(void);



#endif
