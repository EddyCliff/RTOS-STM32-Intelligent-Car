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
float mechanical_zero;        //机械0点
struct _PARMETER	PARMETER;   //定义PID字符串结构体变量

static int Location=0;                            //记录菜单选项的位置
static int Enter_flag=0,exit_flag=0,Write_flag=0; //按键标志位

extern struct pid_arg PID;
/*************************************************************************************************************
*函数名:Key_Scan()
*功能:按键处理函数
*形参:(u8 mode):按键模式(0:单次/1:连续)
*返回值:1:按键1/2:按键2/3:按键3/4:按键4/5:按键5
**************************************************************************************************************/
u8 Key_Scan(u8 mode)
{
	static u8 key_up=1;    //按键松开标志
	if(mode) key_up=1;     //如果是连续按下，那么按键的状态会一直有效

	if(key_up&&(KEY0==0||KEY1==0||KEY2==0||KEY3==0||KEY4==0)){
		HAL_Delay(10);		    //延时去抖动
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
*函数名:Read_KEY_State
*功能:读取按键信息
*形参:(u8 Keynum)
*返回值:无
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
*函数名:Interface()
*功能:用来显示顶层静态界面。
*形参:无
*返回值:无
*说明：  此函数不会被轮询调用，需要的时候调用一次
***************************************************************************************************************/
void Show_Power(void)
{
	switch(power)
	{
		//电量为100%
		case 100: OLED_DrawBMP(110,0,127,1,BMP5);break;
		//电量为75%
		case 75: OLED_DrawBMP(110,0,127,1,BMP4);break;
		//电量为50%
		case 50: OLED_DrawBMP(110,0,127,1,BMP3);break;
		//电量为25%
		case 25: OLED_DrawBMP(110,0,127,1,BMP2);break;
		//电量为0%
		case 0: OLED_DrawBMP(110,0,127,1,BMP1);break;
		
		default:  break;
	}


}


/***************************************************************************************************************
*函数名:Submenu_Display_Parameter()
*功能:参数显示子菜单
*形参:无
*返回值:无
***************************************************************************************************************/

static void Submenu_Display_Parameter(void)
{
			if(Enter_flag==0)
				{
					OLED_ShowC_NMKC(10,0,1,1);//"参数显示"
					OLED_ShowC_NMKC(10,1,2,0);//"模式设置"
					OLED_ShowC_NMKC(10,2,3,0);//"速度设定"
					OLED_ShowC_NMKC(10,3,4,0);//"通信设置"
				}
				if(Enter_flag==1)
				{
					OLED_Clear();//清屏
					//获取机械0点
					mechanical_zero=Mechanical_balance;
					//显示直立环的 P 和 D
					sprintf(PARMETER.Upright_PD,"Stand P:%.0f D:%.1f",PID.Balance_Kp,PID.Balance_Kd);
					//显示速度环的 P 和 I
					sprintf(PARMETER.Speed_PI,"Speed P:%.0f D:%.2f",PID.Velocity_Kp,PID.Velocity_Ki);
					//显示转向环的 P 和 D
					sprintf(PARMETER.Turn_PD,"Turn P:%.0f D:%.2f",PID.Turn_Kp,PID.Turn_Kd);
					//显示机械平衡点Mechanical
					sprintf(PARMETER.Mechanical_Zero,"Mechanical:%.2f",mechanical_zero);
					
					//直立PD值
					OLED_ShowString(0,0,PARMETER.Upright_PD,6);
					//速度PI值
					OLED_ShowString(0,2,PARMETER.Speed_PI,6);
					//转向PD值
					OLED_ShowString(0,4,PARMETER.Turn_PD,6);
					//显示机械平衡点值
					OLED_ShowString(0,6,PARMETER.Mechanical_Zero,6);
	/*+++++++++++++++++++++++++++++++++++++++++二级菜单+++++++++++++++++++++++++++++++++++++++++++++*/
						while(1)
						{
								//读取按键
								key=Key_Scan(0);
								Read_KEY_State(key);
								//捕获到退出按键
								if(exit_flag==1)
								{
										OLED_Clear();//清屏
										Enter_flag=0;
										exit_flag=0;
									break;
								}
						}
				}
}
				
/***************************************************************************************************************
*函数名:Submenu_Set_Mode()
*功能:工作模式设置子菜单
*形参:无
*返回值:无
***************************************************************************************************************/
static void Submenu_Set_WorkMode(void)
{
			if(Enter_flag==0)
			{
					OLED_ShowC_NMKC(10,0,1,0);//"参数显示"
					OLED_ShowC_NMKC(10,1,2,1);//"模式设置"
					OLED_ShowC_NMKC(10,2,3,0);//"速度设定"
					OLED_ShowC_NMKC(10,3,4,0);//"通信设置"
			}
			if(Enter_flag==1)
			{
						OLED_Clear();
						Location=0;
	/*++++++++++++++++++++++++++++++++++++++++二级菜单++++++++++++++++++++++++++++++++++++++++++++++++++++*/
					while(1)
					{
						key=Key_Scan(0);
						Read_KEY_State(key);
						if(Location>=3)		Location=0;
						if(Location<=-1)	Location=2;
						if(Location==0){
							OLED_ShowC_NMKC(10,0,9,1);//"平衡模式"
							OLED_ShowC_NMKC(10,1,11,0);//"避障模式"
							OLED_ShowC_NMKC(10,2,10,0);//"巡线模式"
							
						}
						if(Location==1){
							OLED_ShowC_NMKC(10,0,9,0);//"平衡模式"
							OLED_ShowC_NMKC(10,1,11,1);//"避障模式"
							OLED_ShowC_NMKC(10,2,10,0);//"巡线模式"

						}
						if(Location==2){							
							OLED_ShowC_NMKC(10,0,9,0);//"平衡模式"
							OLED_ShowC_NMKC(10,1,11,0);//"避障模式"	
							OLED_ShowC_NMKC(10,2,10,1);//"巡线模式"
						}	
						//检测到写入按键按下
						if(Write_flag==1){
							Write_flag=0;

							FS_MODE=Location;//模式切换
							
							BUZZ=1;					//蜂鸣器哔哔响
							delay_ms(200);
							BUZZ=0;
						}
						//检测到退出按键按下
						if(exit_flag==1){
							OLED_Clear();//清屏
							Location=0;
							Enter_flag=0;
							exit_flag=0;
							break;
						}
					}
			}
	
}


/***************************************************************************************************************
*函数名:Submenu_Set_Speed()
*功能:速度设置子菜单
*形参:无
*返回值:无
***************************************************************************************************************/
static void Submenu_Set_Speed(void)
{
		if(Enter_flag==0){
					OLED_ShowC_NMKC(10,0,1,0);//"参数显示"
					OLED_ShowC_NMKC(10,1,2,0);//"模式设置"
					OLED_ShowC_NMKC(10,2,3,1);//"速度设定"
					OLED_ShowC_NMKC(10,3,4,0);//"通信设置"
			}
			
			if(Enter_flag==1)
			{
					OLED_Clear();//清屏函数
					OLED_ShowC_NMKC(10,0,7,0);//"速度值"
	
						while(1){
							key=Key_Scan(1);//连续读取按键
							Read_KEY_State(key);
							
							//检测上键按下
							if(key==KEY_UP){
									delay_ms(50);
									Movement++;//速度值+
							}
							//检测下键按下
							if((key==KEY_DOWN) && (Movement>=1)){
								Movement--;//速度值-
								delay_ms(50);
							}
							
							OLED_ShowNum(80,0,Movement,3,8);//显示速度
							
							//检测到写入按键按下
							if(Write_flag==1){
								Write_flag=0;
								//向FLASH发送数组填入数据
								BUZZ=1;
								delay_ms(200);
								BUZZ=0;
							}
						//检测到退出键按下
						if(exit_flag==1){
							OLED_Clear();//清屏
							Enter_flag=0;
							exit_flag=0;
							break;
						}
					}
			}		
}			
			
/***************************************************************************************************************
*函数名:Submenu_Set_NetMode()
*功能:速度设置网络模式
*形参:无
*返回值:无
***************************************************************************************************************/
static void Submenu_Set_NetMode(void)
{
			if(Enter_flag==0){
					OLED_ShowC_NMKC(10,0,1,0);//"参数显示"
					OLED_ShowC_NMKC(10,1,2,0);//"模式设置"
					OLED_ShowC_NMKC(10,2,3,0);//"速度设置"
					OLED_ShowC_NMKC(10,3,4,1);//"通信设置"
			}
			if(Enter_flag==1)
			{
					OLED_Clear();//清屏
				
					while(1){
						key=Key_Scan(0);
						Read_KEY_State(key);
						//"SSID"
						OLED_ShowString(0,0,"SSID:",6);
						//WIFI名字
						OLED_ShowString(18,1,(const char *)("FarsightESP32"),6);
						if(Location>=2)		Location=0;
						if(Location<=-1)	Location=2;
						if(Location==0){
							OLED_ShowC_NMKC(10,2,13,1);//"蓝牙模式"
							OLED_ShowC_NMKC(10,3,12,0);//"WIFI模式"
						}
						if(Location==1){
							OLED_ShowC_NMKC(10,2,13,0);//"蓝牙模式"
							OLED_ShowC_NMKC(10,3,12,1);//"WIFI模式"
						}
							//检测到写入按键按下
					 if(Write_flag==1){
							Write_flag=0;

							BUZZ=1;//蜂鸣器哔哔响
							delay_ms(200);
							BUZZ=0;
					 }
					if(exit_flag==1){
						OLED_Clear();//清屏
						Enter_flag=0;
						exit_flag=0;
						break;
					}
			}
		
		}

}	




/**********************************************************************************************
*函数名:Directory_Show()
*功能:按键菜单
*返回值:无
*形参:无
***********************************************************************************************/
void Directory_Show(void)
{	
		switch(Location){	
			case 0:
					Submenu_Display_Parameter();      //参数显示
					break;
			case 1:
					Submenu_Set_WorkMode();           //模式设置
					break;
			case 2:
					Submenu_Set_Speed();              //速度设定
					break;
			case 3:
					Submenu_Set_NetMode();            //通信设置
					break;
			default:
					if(Location>=4) Location=0;
					if(Location<=-1) Location=3;		
					break;
		}
	
}


/***************************************************************************************************************
*函数名:Interface()
*功能:用来显示顶层静态界面。
*形参:无
*返回值:无
*说明：  此函数不会被轮询调用，需要的时候调用一次
***************************************************************************************************************/
void Interface(void)
{
	//在屏幕上显示Pitch
	OLED_ShowString(0,3,"Pitch:",6);
	//在屏幕上显示Roll
	OLED_ShowString(0,4,"Roll:",6);
	//在屏幕上显示Yaw
	OLED_ShowString(0,5,"Yaw:",6);
	//在屏幕上显示mm
	OLED_ShowString(112,4,"mm",6);
	//在屏幕上显示SPED
	OLED_ShowString(66,5,"SPEED:",6);
	
	//初始化界面，在OLED上显示当前模式
	OLED_ShowString(83,3,"MOD:",6);
	OLED_ShowNum(107,3,FS_MODE,1,6);
	
	//显示"华清远见"
	OLED_ShowC_NMKC(0,0,5,0);

	
	//通信的模式			
		if(NET_MODE == BLE_MODE)
			OLED_DrawBMP(78,0,89,1,BMP10);  //显示蓝牙的图标
		else
			OLED_DrawBMP(78,0,89,1,BMP6);   //显示WIFI的图标
}

/*************************************************************************************************************
*函数名:oled_Show()
*功能:数据显示
*形参：angle(角度值),LEFT\RIGHT(编码器速度值),distance(超声波数值),ccd(CCD数值)
*返回值:无
**************************************************************************************************************/
void oled_Show(void)
{
	
	//显示pitch(俯仰)数值
	OLED_ShowFloat(36,3,OutMpu.pitch,2,6,1);
	//显示roll(翻滚)数值
	OLED_ShowFloat(30,4,OutMpu.roll,2,6,1);
	//显示yaw(偏航)数值
	OLED_ShowFloat(24,5,OutMpu.yaw,2,6,1);
	//超声波数据数值显示
	OLED_ShowFloat(80,4,(Distence*0.58),2,6,1);
	//显示小车右轮速度值
	OLED_ShowFloat(102,5,(float)Encoder_right,3,6,0);

	
	//显示当前WIFI连接状态
	if((WIFI_CONNECT_FLAG!=0)||(BLE_CONNECT_FLAG!=0))
		OLED_DrawBMP(90,0,101,1,BMP8);//连接
	else
		OLED_DrawBMP(90,0,101,1,BMP9);//断开
	
	//显示电量
	Show_Power();
	
	
	//读取按键(单次)
	key=Key_Scan(0);
	if(key==KEY_CENTER){
			OLED_Clear();//清屏

			AIN2(0),			AIN1(0);		//小车左停轮转
			BIN1(0),			BIN2(0);		//小车右轮停转
		
			while(1){
					key=Key_Scan(0);
					Read_KEY_State(key);
				
					//检测退出按键按下
					if(exit_flag==1){
						OLED_Clear();//清屏
						exit_flag=0;
						break;
					}
					Directory_Show();//一级菜单
			}
			
			Interface();    //退出while之后要显示一次静态界面
	}


}
