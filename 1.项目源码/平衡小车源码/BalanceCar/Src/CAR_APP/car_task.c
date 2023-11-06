#include "car_task.h"
#include "mpu6050.h"
#include "inv_mpu_user.h"
#include "contrl.h"
#include "oled.h"
#include "oled_show.h"
#include "car_system.h"

int  FS_MODE = 0 ;                      //0、遥控模式   1、蔽障模式  2、巡线模式 
int  Balance_Pwm,Velocity_Pwm,Turn_Pwm;        //PID计算的PWM值
int  Motor1, Motor2;                  //左右电机PWM值
int  Encoder_left, Encoder_right;     //检测速度
float Movement = 0;                  //速度调节  
int  Contrl_Turn = 64;                //转向调节变量
int  Distence ;                       //小车和前方障碍物之间的距离
struct tCCD  CCD;                      //摄像头的数据
uint8_t   power;                       //定义电池电量


/*************************************************************************************************************
*函数名:Task_200HZ()
*功能:运行主频为200Hz的任务
*形参:无
*返回值:无
*************************************************************************************************************/


//环境数据采集任务
void Car_Task_200HZ(void)
{
		static struct mpu6050_data Last_Data;
	
		if(mpu_dmp_get_data() !=0 )
			OutMpu = Last_Data;
		else
			 Last_Data = OutMpu;
			
}

/**************************************************************************************************************
*函数名:Task_100HZ()
*功能:运行主频为100Hz的任务
*形参:无
*返回值:无
**************************************************************************************************************/

void Car_Task_100HZ(void)
{
	//启动超声波模块检测
	HC_SRC04_Start();
	
	Encoder_left  = Read_Encoder(1);
	Encoder_right = -Read_Encoder(2);
	
	//1、确定直立环PWM
	
		Balance_Pwm = Vertical_Ring_PD(OutMpu.pitch, OutMpu.gyro_x);
	
	//2、确定速度环PWM
	
	  Velocity_Pwm = Vertical_speed_PI(Encoder_left,Encoder_right,OutMpu.pitch, Movement );
	
	
	//3、确定转向环PWM
	
	  if(FS_MODE == 0)       //遥控模式
			Turn_Pwm = Vertical_turn_PD(Contrl_Turn, OutMpu.gyro_z);
		else if(FS_MODE == 1)  //蔽障模式
		{
			if(Distence < 20)
					Turn_Pwm = Vertical_turn_PD(20, OutMpu.gyro_z);
			else
				 Turn_Pwm = 0;
		}
		else if(FS_MODE == 2)  //巡线模式
		{
			   Turn_Pwm = Vertical_turn_PD(CCD.middle, OutMpu.gyro_z);
		}
	
	//4、确定最终左右电机的PWM
		Motor1 = Balance_Pwm + Velocity_Pwm + Turn_Pwm;
	  Motor2 = Balance_Pwm + Velocity_Pwm - Turn_Pwm;
	
		PWM_Limiting(&Motor1,&Motor2);
	
		
		
	
	//5、设置电机
	//判断小车是否出现特殊状况
	if(Turn_off(OutMpu.pitch)==0){
	//输出PWM控制电机
		Set_PWM(Motor1,Motor2);
	}

}


/***************************************************************************************************************
*函数名:Task_20HZ()
*功能:运行主频为20Hz的任务
*形参:无
*返回值:无
***************************************************************************************************************/

void Car_Task_5HZ(void)
{
		 //显示小车状态、任务	
		 Task_State();
		
		
		 //显示菜单 OLED
		 oled_Show();
}

//启动超声波检测
void  HC_SRC04_Start(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	delay_us(20);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	
}
