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
		//�ر�LED  R  G  B��յ��
		case 0:
				HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
		break;
		
		//��LED  R  G  B��յ��, 
		case 1:
				HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);		
		break;
		
		//�ر�LED  G  B��յ��, ����R��
		case 2:
				HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);	
		break;
		
		//�ر�LED  R  B��յ��, ����G��
		case 3:
				HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
		break;
		
		//�ر�LED  R  G��յ��, ����B��
		case 4:
				HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
		break;
		default:
			
		break;
	}

}



/***************************************************************************************************************
*������:Get_battery_volt()
*����:��ȡADCԭʼֵ������ԭʼֵ����������ص�ѹ
*�β�:��
*����ֵ:����ת����ĵ�ص�ѹ,��λ mv 
****************************************************************************************************************/
int Get_battery_volt(void)   
{  
	int  Volt,i,sum=0;//��ص�ѹ
	
	for(i = 0;i<5;i++)
	{
		HAL_ADC_Start(&hadc1);
		
		HAL_ADC_PollForConversion(&hadc1,100);
		
		Volt = HAL_ADC_GetValue(&hadc1)*3300*4/4096;   //�����ѹ���������ԭ��ͼ�򵥷������Եõ�	
		
		sum += Volt;
	}
	
	
	Volt = sum/5;
	//printf("Volt = %d\n",Volt);
		
	return Volt;
}


/***************************************************************************************************************
*������:Task_State()
*����:��ʾС��������״̬
*�β�:��
*����ֵ:��
****************************************************************************************************************/


void Task_State(void)
{
	int voltage;
	
	switch(FS_MODE)
	{
		case 0:              //ң��ģʽ�� R����˸
		  Led_Contrl(2);
			HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
			break;
		
		case 1:
			Led_Contrl(4);    //����ģʽ�� B����˸
			HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
			break;
		
		case 2:
			Led_Contrl(3);     //Ѳ��ģʽ�� G����˸
			HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
			break;
				

		default:
			break;
	}
	
	
	//��ȡС������
	voltage=Get_battery_volt();
	if(voltage>=12000)	{power=100;}	                //����ʣ��100%
	if((voltage<12000)&&(voltage>=11800))	{power=75;}	//����ʣ��75%
	if((voltage<11800)&&(voltage>=11400)) {power=50;}	//����ʣ��50%
	if((voltage<11400)&&(voltage>=11200))	{power=25;}	//����ʣ��25%
	if(voltage<11200){
			power=0;
			if(voltage>9000){
						//��ѹ����
					BUZZ=1;//������������
			}
	}	
}

