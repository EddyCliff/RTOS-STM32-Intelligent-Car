#include "car_system.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "main.h"
#include "car_task.h"
#include "oled_show.h"

void  Led_Contrl(uint8_t cmd)
{
	
	switch(cmd)
	{
		//关闭LED  R  G  B三盏灯
		case 0:
				HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
		break;
		
		//打开LED  R  G  B三盏灯, 
		case 1:
				HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);		
		break;
		
		//关闭LED  G  B三盏灯, 保留R灯
		case 2:
				HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);	
		break;
		
		//关闭LED  R  B三盏灯, 保留G灯
		case 3:
				HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
		break;
		
		//关闭LED  R  G三盏灯, 保留B灯
		case 4:
				HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
		break;
		default:
			
		break;
	}

}



/***************************************************************************************************************
*函数名:Get_battery_volt()
*功能:获取ADC原始值，并对原始值做处理代表电池电压
*形参:无
*返回值:经过转换后的电池电压,单位 mv 
****************************************************************************************************************/
int Get_battery_volt(void)   
{  
	int  Volt,i,sum=0;//电池电压
	
	for(i = 0;i<5;i++)
	{
		HAL_ADC_Start(&hadc1);
		
		HAL_ADC_PollForConversion(&hadc1,100);
		
		Volt = HAL_ADC_GetValue(&hadc1)*3300*4/4096;   //电阻分压，具体根据原理图简单分析可以得到	
		
		sum += Volt;
	}
	
	
	Volt = sum/5;
	//printf("Volt = %d\n",Volt);
		
	return Volt;
}


/***************************************************************************************************************
*函数名:Task_State()
*功能:显示小车的运行状态
*形参:无
*返回值:无
****************************************************************************************************************/


void Task_State(void)
{
	int voltage;
	
	switch(FS_MODE)
	{
		case 0:              //遥控模式， R灯闪烁
		  Led_Contrl(2);
			HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
			break;
		
		case 1:
			Led_Contrl(4);    //蔽障模式， B灯闪烁
			HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
			break;
		
		case 2:
			Led_Contrl(3);     //巡线模式， G灯闪烁
			HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
			break;
				

		default:
			break;
	}
	
	
	//获取小车电量
	voltage=Get_battery_volt();
	if(voltage>=12000)	{power=100;}	                //电量剩余100%
	if((voltage<12000)&&(voltage>=11800))	{power=75;}	//电量剩余75%
	if((voltage<11800)&&(voltage>=11400)) {power=50;}	//电量剩余50%
	if((voltage<11400)&&(voltage>=11200))	{power=25;}	//电量剩余25%
	if(voltage<11200){
			power=0;
			if(voltage>9000){
						//低压报警
					BUZZ=1;//蜂鸣器哔哔响
			}
	}	
}

