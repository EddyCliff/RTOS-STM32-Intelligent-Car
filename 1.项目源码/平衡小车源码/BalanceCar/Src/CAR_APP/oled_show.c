#include "oled_show.h"
#include "connect.h"
#include "car_task.h"
#include "stm32f4xx_hal.h"
#include "oled.h"
#include "inv_mpu_user.h"
#include "esp32.h"
#include "bmp.h"
#include "contrl.h"
#include "inv_mpu.h"

u8 key;
float mechanical_zero;        //��е0��
struct _PARMETER	PARMETER;   //����PID�ַ����ṹ�����

static int Location=0;                            //��¼�˵�ѡ���λ��
static int Enter_flag=0,exit_flag=0,Write_flag=0; //������־λ

extern struct pid_arg PID;
/*************************************************************************************************************
*������:Key_Scan()
*����:����������
*�β�:(u8 mode):����ģʽ(0:����/1:����)
*����ֵ:1:����1/2:����2/3:����3/4:����4/5:����5
**************************************************************************************************************/
u8 Key_Scan(u8 mode)
{
	static u8 key_up=1;    //�����ɿ���־
	if(mode) key_up=1;     //������������£���ô������״̬��һֱ��Ч

	if(key_up&&(KEY0==0||KEY1==0||KEY2==0||KEY3==0||KEY4==0)){
		HAL_Delay(10);		    //��ʱȥ����
		key_up=0;
		
	  if(KEY0==0)return 1;
		else if(KEY1==0)return 2;
		else if(KEY2==0)return 3;
		else if(KEY3==0)return 4;
		else if(KEY4==0)return 5;
	}else if(KEY0==1&&KEY1==1&&KEY2==1&&KEY3==1&&KEY4==1)key_up=1;
	return 0;
}

/*************************************************************************************************************
*������:Read_KEY_State
*����:��ȡ������Ϣ
*�β�:(u8 Keynum)
*����ֵ:��
**************************************************************************************************************/
void Read_KEY_State(u8 Keynum)
{
	if(key){
	switch(key){	
		case KEY_LEFT:
				exit_flag=1;
				break;
		case KEY_UP:
				Location--;
				break;
		case KEY_DOWN:
				Location++;
				break;
		case KEY_CENTER:
				Write_flag=1;
				break;
		case KEY_RIGHT:
				Enter_flag=1;
				break;
		}
	}
}

/***************************************************************************************************************
*������:Interface()
*����:������ʾ���㾲̬���档
*�β�:��
*����ֵ:��
*˵����  �˺������ᱻ��ѯ���ã���Ҫ��ʱ�����һ��
***************************************************************************************************************/
void Show_Power(void)
{
	switch(power)
	{
		//����Ϊ100%
		case 100: OLED_DrawBMP(110,0,127,1,BMP5);break;
		//����Ϊ75%
		case 75: OLED_DrawBMP(110,0,127,1,BMP4);break;
		//����Ϊ50%
		case 50: OLED_DrawBMP(110,0,127,1,BMP3);break;
		//����Ϊ25%
		case 25: OLED_DrawBMP(110,0,127,1,BMP2);break;
		//����Ϊ0%
		case 0: OLED_DrawBMP(110,0,127,1,BMP1);break;
		
		default:  break;
	}


}


/***************************************************************************************************************
*������:Submenu_Display_Parameter()
*����:������ʾ�Ӳ˵�
*�β�:��
*����ֵ:��
***************************************************************************************************************/

static void Submenu_Display_Parameter(void)
{
			if(Enter_flag==0)
				{
					OLED_ShowC_NMKC(10,0,1,1);//"������ʾ"
					OLED_ShowC_NMKC(10,1,2,0);//"ģʽ����"
					OLED_ShowC_NMKC(10,2,3,0);//"�ٶ��趨"
					OLED_ShowC_NMKC(10,3,4,0);//"ͨ������"
				}
				if(Enter_flag==1)
				{
					OLED_Clear();//����
					//��ȡ��е0��
					mechanical_zero=Mechanical_balance;
					//��ʾֱ������ P �� D
					sprintf(PARMETER.Upright_PD,"Stand P:%.0f D:%.1f",PID.Balance_Kp,PID.Balance_Kd);
					//��ʾ�ٶȻ��� P �� I
					sprintf(PARMETER.Speed_PI,"Speed P:%.0f D:%.2f",PID.Velocity_Kp,PID.Velocity_Ki);
					//��ʾת�򻷵� P �� D
					sprintf(PARMETER.Turn_PD,"Turn P:%.0f D:%.2f",PID.Turn_Kp,PID.Turn_Kd);
					//��ʾ��еƽ���Mechanical
					sprintf(PARMETER.Mechanical_Zero,"Mechanical:%.2f",mechanical_zero);
					
					//ֱ��PDֵ
					OLED_ShowString(0,0,PARMETER.Upright_PD,6);
					//�ٶ�PIֵ
					OLED_ShowString(0,2,PARMETER.Speed_PI,6);
					//ת��PDֵ
					OLED_ShowString(0,4,PARMETER.Turn_PD,6);
					//��ʾ��еƽ���ֵ
					OLED_ShowString(0,6,PARMETER.Mechanical_Zero,6);
	/*+++++++++++++++++++++++++++++++++++++++++�����˵�+++++++++++++++++++++++++++++++++++++++++++++*/
						while(1)
						{
								//��ȡ����
								key=Key_Scan(0);
								Read_KEY_State(key);
								//�����˳�����
								if(exit_flag==1)
								{
										OLED_Clear();//����
										Enter_flag=0;
										exit_flag=0;
									break;
								}
						}
				}
}
				
/***************************************************************************************************************
*������:Submenu_Set_Mode()
*����:����ģʽ�����Ӳ˵�
*�β�:��
*����ֵ:��
***************************************************************************************************************/
static void Submenu_Set_WorkMode(void)
{
			if(Enter_flag==0)
			{
					OLED_ShowC_NMKC(10,0,1,0);//"������ʾ"
					OLED_ShowC_NMKC(10,1,2,1);//"ģʽ����"
					OLED_ShowC_NMKC(10,2,3,0);//"�ٶ��趨"
					OLED_ShowC_NMKC(10,3,4,0);//"ͨ������"
			}
			if(Enter_flag==1)
			{
						OLED_Clear();
						Location=0;
	/*++++++++++++++++++++++++++++++++++++++++�����˵�++++++++++++++++++++++++++++++++++++++++++++++++++++*/
					while(1)
					{
						key=Key_Scan(0);
						Read_KEY_State(key);
						if(Location>=3)		Location=0;
						if(Location<=-1)	Location=2;
						if(Location==0){
							OLED_ShowC_NMKC(10,0,9,1);//"ƽ��ģʽ"
							OLED_ShowC_NMKC(10,1,11,0);//"����ģʽ"
							OLED_ShowC_NMKC(10,2,10,0);//"Ѳ��ģʽ"
							
						}
						if(Location==1){
							OLED_ShowC_NMKC(10,0,9,0);//"ƽ��ģʽ"
							OLED_ShowC_NMKC(10,1,11,1);//"����ģʽ"
							OLED_ShowC_NMKC(10,2,10,0);//"Ѳ��ģʽ"

						}
						if(Location==2){							
							OLED_ShowC_NMKC(10,0,9,0);//"ƽ��ģʽ"
							OLED_ShowC_NMKC(10,1,11,0);//"����ģʽ"	
							OLED_ShowC_NMKC(10,2,10,1);//"Ѳ��ģʽ"
						}	
						//��⵽д�밴������
						if(Write_flag==1){
							Write_flag=0;

							FS_MODE=Location;//ģʽ�л�
							
							BUZZ=1;					//������������
							delay_ms(200);
							BUZZ=0;
						}
						//��⵽�˳���������
						if(exit_flag==1){
							OLED_Clear();//����
							Location=0;
							Enter_flag=0;
							exit_flag=0;
							break;
						}
					}
			}
	
}


/***************************************************************************************************************
*������:Submenu_Set_Speed()
*����:�ٶ������Ӳ˵�
*�β�:��
*����ֵ:��
***************************************************************************************************************/
static void Submenu_Set_Speed(void)
{
		if(Enter_flag==0){
					OLED_ShowC_NMKC(10,0,1,0);//"������ʾ"
					OLED_ShowC_NMKC(10,1,2,0);//"ģʽ����"
					OLED_ShowC_NMKC(10,2,3,1);//"�ٶ��趨"
					OLED_ShowC_NMKC(10,3,4,0);//"ͨ������"
			}
			
			if(Enter_flag==1)
			{
					OLED_Clear();//��������
					OLED_ShowC_NMKC(10,0,7,0);//"�ٶ�ֵ"
	
						while(1){
							key=Key_Scan(1);//������ȡ����
							Read_KEY_State(key);
							
							//����ϼ�����
							if(key==KEY_UP){
									delay_ms(50);
									Movement++;//�ٶ�ֵ+
							}
							//����¼�����
							if((key==KEY_DOWN) && (Movement>=1)){
								Movement--;//�ٶ�ֵ-
								delay_ms(50);
							}
							
							OLED_ShowNum(80,0,Movement,3,8);//��ʾ�ٶ�
							
							//��⵽д�밴������
							if(Write_flag==1){
								Write_flag=0;
								//��FLASH����������������
								BUZZ=1;
								delay_ms(200);
								BUZZ=0;
							}
						//��⵽�˳�������
						if(exit_flag==1){
							OLED_Clear();//����
							Enter_flag=0;
							exit_flag=0;
							break;
						}
					}
			}		
}			
			
/***************************************************************************************************************
*������:Submenu_Set_NetMode()
*����:�ٶ���������ģʽ
*�β�:��
*����ֵ:��
***************************************************************************************************************/
static void Submenu_Set_NetMode(void)
{
			if(Enter_flag==0){
					OLED_ShowC_NMKC(10,0,1,0);//"������ʾ"
					OLED_ShowC_NMKC(10,1,2,0);//"ģʽ����"
					OLED_ShowC_NMKC(10,2,3,0);//"�ٶ�����"
					OLED_ShowC_NMKC(10,3,4,1);//"ͨ������"
			}
			if(Enter_flag==1)
			{
					OLED_Clear();//����
				
					while(1){
						key=Key_Scan(0);
						Read_KEY_State(key);
						//"SSID"
						OLED_ShowString(0,0,"SSID:",6);
						//WIFI����
						OLED_ShowString(18,1,(const char *)("FarsightESP32"),6);
						if(Location>=2)		Location=0;
						if(Location<=-1)	Location=2;
						if(Location==0){
							OLED_ShowC_NMKC(10,2,13,1);//"����ģʽ"
							OLED_ShowC_NMKC(10,3,12,0);//"WIFIģʽ"
						}
						if(Location==1){
							OLED_ShowC_NMKC(10,2,13,0);//"����ģʽ"
							OLED_ShowC_NMKC(10,3,12,1);//"WIFIģʽ"
						}
							//��⵽д�밴������
					 if(Write_flag==1){
							Write_flag=0;

							BUZZ=1;//������������
							delay_ms(200);
							BUZZ=0;
					 }
					if(exit_flag==1){
						OLED_Clear();//����
						Enter_flag=0;
						exit_flag=0;
						break;
					}
			}
		
		}

}	




/**********************************************************************************************
*������:Directory_Show()
*����:�����˵�
*����ֵ:��
*�β�:��
***********************************************************************************************/
void Directory_Show(void)
{	
		switch(Location){	
			case 0:
					Submenu_Display_Parameter();      //������ʾ
					break;
			case 1:
					Submenu_Set_WorkMode();           //ģʽ����
					break;
			case 2:
					Submenu_Set_Speed();              //�ٶ��趨
					break;
			case 3:
					Submenu_Set_NetMode();            //ͨ������
					break;
			default:
					if(Location>=4) Location=0;
					if(Location<=-1) Location=3;		
					break;
		}
	
}


/***************************************************************************************************************
*������:Interface()
*����:������ʾ���㾲̬���档
*�β�:��
*����ֵ:��
*˵����  �˺������ᱻ��ѯ���ã���Ҫ��ʱ�����һ��
***************************************************************************************************************/
void Interface(void)
{
	//����Ļ����ʾPitch
	OLED_ShowString(0,3,"Pitch:",6);
	//����Ļ����ʾRoll
	OLED_ShowString(0,4,"Roll:",6);
	//����Ļ����ʾYaw
	OLED_ShowString(0,5,"Yaw:",6);
	//����Ļ����ʾmm
	OLED_ShowString(112,4,"mm",6);
	//����Ļ����ʾSPED
	OLED_ShowString(66,5,"SPEED:",6);
	
	//��ʼ�����棬��OLED����ʾ��ǰģʽ
	OLED_ShowString(83,3,"MOD:",6);
	OLED_ShowNum(107,3,FS_MODE,1,6);
	
	//��ʾ"����Զ��"
	OLED_ShowC_NMKC(0,0,5,0);

	
	//ͨ�ŵ�ģʽ			
		if(NET_MODE == BLE_MODE)
			OLED_DrawBMP(78,0,89,1,BMP10);  //��ʾ������ͼ��
		else
			OLED_DrawBMP(78,0,89,1,BMP6);   //��ʾWIFI��ͼ��
}

/*************************************************************************************************************
*������:oled_Show()
*����:������ʾ
*�βΣ�angle(�Ƕ�ֵ),LEFT\RIGHT(�������ٶ�ֵ),distance(��������ֵ),ccd(CCD��ֵ)
*����ֵ:��
**************************************************************************************************************/
void oled_Show(void)
{
	
	//��ʾpitch(����)��ֵ
	OLED_ShowFloat(36,3,OutMpu.pitch,2,6,1);
	//��ʾroll(����)��ֵ
	OLED_ShowFloat(30,4,OutMpu.roll,2,6,1);
	//��ʾyaw(ƫ��)��ֵ
	OLED_ShowFloat(24,5,OutMpu.yaw,2,6,1);
	//������������ֵ��ʾ
	OLED_ShowFloat(80,4,(Distence*0.58),2,6,1);
	//��ʾС�������ٶ�ֵ
	OLED_ShowFloat(102,5,(float)Encoder_right,3,6,0);

	
	//��ʾ��ǰWIFI����״̬
	if((WIFI_CONNECT_FLAG!=0)||(BLE_CONNECT_FLAG!=0))
		OLED_DrawBMP(90,0,101,1,BMP8);//����
	else
		OLED_DrawBMP(90,0,101,1,BMP9);//�Ͽ�
	
	//��ʾ����
	Show_Power();
	
	
	//��ȡ����(����)
	key=Key_Scan(0);
	if(key==KEY_CENTER){
			OLED_Clear();//����

			AIN2(0),			AIN1(0);		//С����ͣ��ת
			BIN1(0),			BIN2(0);		//С������ͣת
		
			while(1){
					key=Key_Scan(0);
					Read_KEY_State(key);
				
					//����˳���������
					if(exit_flag==1){
						OLED_Clear();//����
						exit_flag=0;
						break;
					}
					Directory_Show();//һ���˵�
			}
			
			Interface();    //�˳�while֮��Ҫ��ʾһ�ξ�̬����
	}


}
