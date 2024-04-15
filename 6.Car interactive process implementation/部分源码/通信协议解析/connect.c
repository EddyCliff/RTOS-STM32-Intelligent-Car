#include "inv_mpu_user.h"
#include "connect.h"
#include "string.h"
#include "stdio.h"
#include "esp32.h"
#include "car_task.h"
#include "contrl.h"

/**************************************************************************************************************
*函数名:package_report_data()
*功能:添加帧头功能字及校验位
*形参:(u8 fun):功能字/(u8*data):要发送的数据包/(u8 len):长度
*返回值:无
**************************************************************************************************************/
static void package_report_data(u8 fun,u8*data,u8 len)
{
    static u8 send_buf[40]={0};   //添加static，给栈减小一点压力
		u16 check_sum=0;
    u8 i;
		
		
		//封装协议头
    if(len>28)return;   
    send_buf[0]=0XAA;  
		send_buf[1]=0XAA; 
    send_buf[2]=fun;    
    send_buf[3]=len; 

		//封装数据
    for(i=0;i<len;i++)send_buf[4+i]=data[i];

		//计算校验值
    for(i=0;i<len+4;i++)	check_sum+=send_buf[i];	
		send_buf[len+4]=((check_sum)&0xFF);
		
		//发送数据
		ESP32_Send_Data(send_buf,len+5);
}

/*********************************************************************************************************
*函数名:ANO_VER()
*功能：发送版本信息
*形参:HardwareType,HardwareVER,SoftwareVER,ProtocolVER
*返回值:无
*********************************************************************************************************/
static void ANO_VER(u8 HardwareType,u16 HardwareVER,u16 SoftwareVER,u16 ProtocolVER)
{
		static u8 tbuf[7];   
		tbuf[0]=(HardwareType)&0XFF;
	
		tbuf[1]=((HardwareVER)>>8)&0XFF;
		tbuf[2]=(HardwareVER)&0XFF;
	
		tbuf[3]=((SoftwareVER)>>8)&0XFF;
		tbuf[4]=(SoftwareVER)&0XFF;
	
		tbuf[5]=((ProtocolVER)>>8)&0XFF;
		tbuf[6]=(ProtocolVER)&0XFF;
		package_report_data(CAR_VER,tbuf,7);
}

/*************************************************************************************************************
*函数名：ANO_MOD()
*功能:发送当前模式
*形参:data模式
*返回值:无
*************************************************************************************************************/
static void ANO_MOD(u8 data)
{
	u8 tbuf[1];
	tbuf[0]=(data)&0XFF;  
	package_report_data(CAR_POSE,tbuf,1);
}
/**********************************************************************************************************
*函数名:ANO_CAR_POSE()
*功能:发送姿态
*形参:angle(x俯仰,y横滚,z偏航)
*返回值:无
***********************************************************************************************************/
static void ANO_CAR_POSE(void)
{
	static u8 tbuf[6]; 
	int32_t anglex,angley,anglez;
	anglex=((short)(OutMpu.pitch))*100;
	angley=((short)(OutMpu.roll))*100;
	anglez=((short)(OutMpu.yaw))*100;
	
	tbuf[0]=((anglex)>>8)&0XFF;      
	tbuf[1]=(anglex)&0XFF;
	tbuf[2]=((angley)>>8)&0XFF;
	tbuf[3]=(angley)&0XFF;
	tbuf[4]=((anglez)>>8)&0XFF;
	tbuf[5]=(anglez)&0XFF;
	package_report_data(CAR_POSE,tbuf,6);
}

/**************************************************************************************************************
*函数名:ANO_SENSER()
*功能:发送传感器原数据
*形参:acc:陀螺仪,gyro:加速度计,mag:电子罗盘
*返回值:无
**************************************************************************************************************/
static void ANO_SENSER(void)
{
	static u8 tbuf[18];    
	u8 accx,accy,accz,gyrox,gyroy,gyroz,magx,magy,magz;
	
	accx=(u8)((OutMpu.acc_x)*100);
	accy=(u8)((OutMpu.acc_y)*100);
	accz=(u8)((OutMpu.acc_z)*100);
	
	gyrox=(u8)((OutMpu.gyro_x)*100);
	gyroy=(u8)((OutMpu.gyro_x)*100);
	gyroz=(u8)((OutMpu.gyro_x)*100);
	
//	magx=(u8)((mag->x)*100);
//	magy=(u8)((mag->y)*100);
//	magz=(u8)((mag->z)*100);
	
	tbuf[0]=((accx)>>8)&0XFF;      
	tbuf[1]=(accx)&0XFF;
	tbuf[2]=((accy)>>8)&0XFF;
	tbuf[3]=(accy)&0XFF;
	tbuf[4]=((accz)>>8)&0XFF;
	tbuf[5]=(accz)&0XFF;
	
	tbuf[6]=((gyrox)>>8)&0XFF;      
	tbuf[7]=(gyrox)&0XFF;
	tbuf[8]=((gyroy)>>8)&0XFF;
	tbuf[9]=(gyroy)&0XFF;
	tbuf[10]=((gyroz)>>8)&0XFF;
	tbuf[11]=(gyroz)&0XFF;
	
	tbuf[12]=((magx)>>8)&0XFF;      
	tbuf[13]=(magx)&0XFF;
	tbuf[14]=((magy)>>8)&0XFF;
	tbuf[15]=(magy)&0XFF;
	tbuf[16]=((magz)>>8)&0XFF;
	tbuf[17]=(magz)&0XFF;
	package_report_data(CAR_SENSER,tbuf,18);
}
/*************************************************************************************************************
*函数名:ANO_PID()
*功能:发送PID数据
*形参:PID1，PID2，PID3的参数
*返回值:无
**************************************************************************************************************/
static void ANO_PID(u8 Function)
{
  static u8 tbuf[18];
	int16_t	PID1_P,PID1_I,PID1_D,PID2_P,PID2_I,PID2_D,PID3_P,PID3_I,PID3_D;
	
	PID1_P=(u16)((PID.Balance_Kp)*100);
	PID1_I=(u16)((PID.Balance_Ki)*100);
	PID1_D=(u16)((PID.Balance_Kd)*100);
	
	PID2_P=(u16)((PID.Velocity_Kp)*100);
	PID2_I=(u16)((PID.Velocity_Ki)*100);
	PID2_D=(u16)((PID.Velocity_Kd)*100);
	
	PID3_P=(u16)((PID.Turn_Kp)*100);
	PID3_I=(u16)((PID.Turn_Ki)*100);
	PID3_D=(u16)((PID.Turn_Kd)*100);
	
	tbuf[0]=((PID1_P)>>8)&0XFF;      
	tbuf[1]=(PID1_P)&0XFF;
	tbuf[2]=((PID1_I)>>8)&0XFF;
	tbuf[3]=(PID1_I)&0XFF;
	tbuf[4]=((PID1_D)>>8)&0XFF;
	tbuf[5]=(PID1_D)&0XFF;
		
	tbuf[6]=((PID2_P)>>8)&0XFF;      
	tbuf[7]=(PID2_P)&0XFF;
	tbuf[8]=((PID2_I)>>8)&0XFF;
	tbuf[9]=(PID2_I)&0XFF;
	tbuf[10]=((PID2_D)>>8)&0XFF;
	tbuf[11]=(PID2_D)&0XFF;
	
	tbuf[12]=((PID3_P)>>8)&0XFF;      
	tbuf[13]=(PID3_P)&0XFF;
	tbuf[14]=((PID3_I)>>8)&0XFF;
	tbuf[15]=(PID3_I)&0XFF;
	tbuf[16]=((PID3_D)>>8)&0XFF;
	tbuf[17]=(PID3_D)&0XFF;
	if(Function==1)
		package_report_data(CAR_PID_1,tbuf,18);
	if(Function==2)
		package_report_data(CAR_PID_2,tbuf,18);
	if(Function==3)
		package_report_data(CAR_PID_1,tbuf,18);
}

/**************************************************************************************************************
*函数名:ANO_CCD_SEN()
*功能:发送CCD数据
*形参:ccd:CCD数据包括(阈值,中值,左跳变,右跳变)
*返回值:无
***************************************************************************************************************/
void	ANO_CCD_SEN(void)
{
	static u8 tbuf[8];
	u16 CCD_MIDDLE,CCD_THRESHOLD,CCD_LEFT,CCD_RIGHT;
	
	CCD_MIDDLE=CCD.middle;
	CCD_THRESHOLD=CCD.threshold;
	CCD_LEFT=CCD.left;
	CCD_RIGHT=CCD.right;
	
	tbuf[0]=((CCD_MIDDLE)>>8)&0XFF;
	tbuf[1]=(CCD_MIDDLE)&0XFF;
	
	tbuf[2]=((CCD_THRESHOLD)>>8)&0XFF;
	tbuf[3]=(CCD_THRESHOLD)&0XFF;
	
	tbuf[4]=((CCD_LEFT)>>8)&0XFF;
	tbuf[5]=(CCD_LEFT)&0XFF;
	
	tbuf[6]=((CCD_RIGHT)>>8)&0XFF;
	tbuf[7]=(CCD_RIGHT)&0XFF;
	
	package_report_data(CAR_CCD_SEN,tbuf,8);
}

/*************************************************************************************************************
*函数名:ANO_POWER()
*功能:发送电量
*形参:data:电量(0%,25%,50%,75%,100%)
*返回值:
**************************************************************************************************************/
void ANO_POWER(u16 data)
{
	static u8 tbuf[2];
	tbuf[0]=((data)>>8)&0XFF;      
	tbuf[1]=(data)&0XFF;
	package_report_data(CAR_POWER,tbuf,2);
}
/*************************************************************************************************************
*函数名:ANO_MOTO()
*功能:发送电机转速
*形参:PWM_MOTO:电机转速
*返回值:无
**************************************************************************************************************/
void ANO_MOTO(float PWM_MOTO)
{
	u16 PWM_Percentage;
	static u8 tbuf[2];
	
	PWM_Percentage=fabs(PWM_MOTO)*1.24;
	
	tbuf[0]=((PWM_Percentage)>>8)&0XFF;      
	tbuf[1]=(PWM_Percentage)&0XFF;
	package_report_data(CAR_MOTO,tbuf,2);
}
/**************************************************************************************************************
*函数名:Connect_Send_data()
*功能：发送数据给上位机
*形参:
*返回值:无
**************************************************************************************************************/
void Connect_Send_data(u8 CMD_Data)
{
	
		  switch(CMD_Data)
			{
				case READ_ALL_ARG:
				//			DBG("\r发送数据到上位机\n");
	
//							ANO_VER(Hardware_Type,Hardware_VER,Software_VER,Protocol_VER);
//							ANO_MOD(FS_MODE);
							ANO_CAR_POSE();
							ANO_SENSER();
//							ANO_RCDATA(F_CMD,Movement);
//							ANO_POWER(power);
							ANO_MOTO(Encoder_right);
//							ANO_SENSER2(Distance);
							ANO_CCD_SEN();
				
					break;
				case READ_PID:
						DBG("\r发送PID请求\n");
				
						ANO_PID(1);
					break;
				case READ_WORK_MODE:
					  DBG("\r发送当前模式\n");
					  ANO_MOD(FS_MODE);
					break;
				case READ_VERINFO:
					DBG("\r发送版本信息\n");
				  
				  ANO_VER(Hardware_Type,Hardware_VER,Software_VER,Protocol_VER);
					break;
				default:
					break;
			}

}




/**************************************************************************************************************
*函数名:Host_Data_Receive_Anl()
*功能：上位机数据包解析
*形参:(u8 *data_buf):缓存区中的接收到的数据/(u8 num):数据包长度
*返回值:无
**************************************************************************************************************/
void Host_Data_Receive_Anl(u8 *data_buf,u8 num)
{
	u8 sum = 0,i;
	u8 function = *(data_buf+2),cmd = *(data_buf+4);

	
	//从帧头道数据位的计算校验；
	DBG("收到有效数据：");
	for(i=0;i<(num-1);i++)
	{
		
		DBG("%x  ",*(data_buf+i));
		sum += *(data_buf+i);
	}
	DBG("\r\n");
	
	////校验不过直接返回
	if(sum != *(data_buf+num-1) )		return;		
	//判断帧头
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;	

	
	switch(function)
	{
		case MOVEMENT: 	
			DBG("设置小车动作\n");
		
		  switch(cmd)
			{
				case CAR_STOP:
					DBG("小车停止\n");
				
					if(FS_MODE==0){
						Contrl_Turn=64;
						Movement=0;
					}
								
					break;
				case CAR_FORWARD:
					DBG("小车前进\n");
				
					if(FS_MODE==0){
						Contrl_Turn=64;
						Movement=50;
					}
										
					break;
				case CAR_BACK:
					DBG("小车后退\n");

					if(FS_MODE==0){
						Contrl_Turn = 64;
						Movement = -50;
					}
					
					break;
				case CAR_TURN_LEFT:
					DBG("小车左转\n");

					if(FS_MODE==0){
						Contrl_Turn=30;
						Movement=0;
					}
					
					break;
				case CAR_TURN_RIGHT:
					DBG("小车右转\n");

					if(FS_MODE==0){
						Contrl_Turn=98;
						Movement=0;
					}
					
					break;
				default:
					break;
			}
			break;
			
		case READINFO: 
			DBG("读取小车基本信息\n");

		  switch(cmd)
			{
				case READ_PID:
					DBG("读取PID数据\n");
				
					break;
				case READ_WORK_MODE:
					DBG("读取当前工作模式\n");
				
					break;
				case READ_VERINFO:
					DBG("读取版本信息\n");
				
					break;
				case RESTORE_DEFAULT_ARG:
					DBG("恢复默认参数\n");
				
						PID.Balance_Kp=200;
						PID.Balance_Kd=1;
						PID.Velocity_Kp=-60;
						PID.Velocity_Ki=-0.3;
						PID.Turn_Kp = 18;
						PID.Turn_Kd = 0.18;

					break;			
				default:
					break;
			}	
		
			break;
		case SETMODE: 
			DBG("设置小车工作模式\n");
			
		  switch(cmd)
			{
				case REMOTE_MODE:
					DBG("遥控模式\n");
					
					FS_MODE = 0;
				
					break;
				case AVOID_MODE:
					DBG("避障模式\n");
				
					FS_MODE = 1;
					break;
				case LINE_TRACK_MODE:
					DBG("巡线模式\n");
				
					FS_MODE = 2;
				
					break;

				default:
					break;
			}		
		
			break;
		case WRITEPID1: 
			//上位机发送9个的16位数，*100后发送过来的,
			DBG("设置小车PID1\n");
		
			PID.Balance_Kp  = 0.01*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
			PID.Balance_Ki  = 0.01*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
			PID.Balance_Kd  = 0.01*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
			
			PID.Velocity_Kp = 0.01*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
			PID.Velocity_Ki = 0.01*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
			PID.Velocity_Kd = 0.01*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
			
			PID.Turn_Kp 	= 0.01*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
			PID.Turn_Ki 	= 0.01*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
			PID.Turn_Kd	  = 0.01*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );

			break;
		case WRITEPID2: 
			DBG("设置小车PID2\n");
		
			break;
		default:
			break;
		
	}
	
}


/**********************************************************************************************************
*函数名:EP32_RcvData_Extract
*功能:提取ESP32中有效数据,并且将其拷贝到缓存区中
*形参:中断接收的数据
*返回值:成功返回0，失败返回1
注：数据格式
							命令  帧头  帧头  功能  长度  数据  校验
例：          前进  0xaa  0xaf  0x01  0x01  0x01  计算
***********************************************************************************************************/

u8 EP32_RcvData_Extract(const uint8_t *Buff,int len)
{	
	static u8 RxBuffer[50];
	static int data_len = 0,data_cnt = 0;
	static u8 state = 0;
	u8 i, data;
	
	
	for(i=0;i<len ;i++){
		  
		  data = Buff[i];
		
		   //判断帧头		
			if(state==0&&data==0xAA)
			{
				state=1;
				RxBuffer[0]=data;
			}
			else if(state==1&&data==0xAF)
			{
				state=2;
				RxBuffer[1]=data;
			}
			//功能字截止到0XF1
			else if(state==2&&data<0XF1)
			{
				state=3;
				RxBuffer[2]=data;
			}
			//数据长度小于50
				else if(state==3&&data<50)
			{
				state = 4;
				RxBuffer[3]=data;
				data_len = data;
				data_cnt = 0;
			}
			//将数据复制到RxBuffer中
			else if(state==4&&data_len>0)
			{
				data_len--;
				RxBuffer[4+data_cnt++]=data;
				if(data_len==0)
					state = 5;
			}
			//计算校验值
			else if(state==5)
			{
				state = 0;
				RxBuffer[4+data_cnt]=data;
				Host_Data_Receive_Anl(RxBuffer,data_cnt+5);
				memset(RxBuffer,0,50);
				return 0;
			}

	}
	return 1;
}
