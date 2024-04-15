#include "car_task.h"
#include "mpu6050.h"
#include "inv_mpu_user.h"
#include "contrl.h"
#include "oled.h"
#include "oled_show.h"
#include "car_system.h"

int  FS_MODE = 0 ;                      //0��ң��ģʽ   1������ģʽ  2��Ѳ��ģʽ 
int  Balance_Pwm,Velocity_Pwm,Turn_Pwm;        //PID�����PWMֵ
int  Motor1, Motor2;                  //���ҵ��PWMֵ
int  Encoder_left, Encoder_right;     //����ٶ�
float Movement = 0;                  //�ٶȵ���  
int  Contrl_Turn = 64;                //ת����ڱ���
int  Distence ;                       //С����ǰ���ϰ���֮��ľ���
struct tCCD  CCD;                      //����ͷ������
uint8_t   power;                       //�����ص���


/*************************************************************************************************************
*������:Task_200HZ()
*����:������ƵΪ200Hz������
*�β�:��
*����ֵ:��
*************************************************************************************************************/


//�������ݲɼ�����
void Car_Task_200HZ(void)
{
		static struct mpu6050_data Last_Data;
	
		if(mpu_dmp_get_data() !=0 )
			OutMpu = Last_Data;
		else
			 Last_Data = OutMpu;
			
}

/**************************************************************************************************************
*������:Task_100HZ()
*����:������ƵΪ100Hz������
*�β�:��
*����ֵ:��
**************************************************************************************************************/

void Car_Task_100HZ(void)
{
	//����������ģ����
	HC_SRC04_Start();
	
	Encoder_left  = Read_Encoder(1);
	Encoder_right = -Read_Encoder(2);
	
	//1��ȷ��ֱ����PWM
	
		Balance_Pwm = Vertical_Ring_PD(OutMpu.pitch, OutMpu.gyro_x);
	
	//2��ȷ���ٶȻ�PWM
	
	  Velocity_Pwm = Vertical_speed_PI(Encoder_left,Encoder_right,OutMpu.pitch, Movement );
	
	
	//3��ȷ��ת��PWM
	
	  if(FS_MODE == 0)       //ң��ģʽ
			Turn_Pwm = Vertical_turn_PD(Contrl_Turn, OutMpu.gyro_z);
		else if(FS_MODE == 1)  //����ģʽ
		{
			if(Distence < 20)
					Turn_Pwm = Vertical_turn_PD(20, OutMpu.gyro_z);
			else
				 Turn_Pwm = 0;
		}
		else if(FS_MODE == 2)  //Ѳ��ģʽ
		{
			   Turn_Pwm = Vertical_turn_PD(CCD.middle, OutMpu.gyro_z);
		}
	
	//4��ȷ���������ҵ����PWM
		Motor1 = Balance_Pwm + Velocity_Pwm + Turn_Pwm;
	  Motor2 = Balance_Pwm + Velocity_Pwm - Turn_Pwm;
	
		PWM_Limiting(&Motor1,&Motor2);
	
		
		
	
	//5�����õ��
	//�ж�С���Ƿ��������״��
	if(Turn_off(OutMpu.pitch)==0){
	//���PWM���Ƶ��
		Set_PWM(Motor1,Motor2);
	}

}


/***************************************************************************************************************
*������:Task_20HZ()
*����:������ƵΪ20Hz������
*�β�:��
*����ֵ:��
***************************************************************************************************************/

void Car_Task_5HZ(void)
{
		 //��ʾС��״̬������	
		 Task_State();
		
		
		 //��ʾ�˵� OLED
		 oled_Show();
}

//�������������
void  HC_SRC04_Start(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	delay_us(20);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	
}
