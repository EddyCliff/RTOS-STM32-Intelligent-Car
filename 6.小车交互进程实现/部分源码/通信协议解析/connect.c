#include "inv_mpu_user.h"
#include "connect.h"
#include "string.h"
#include "stdio.h"
#include "esp32.h"
#include "car_task.h"
#include "contrl.h"

/**************************************************************************************************************
*������:package_report_data()
*����:���֡ͷ�����ּ�У��λ
*�β�:(u8 fun):������/(u8*data):Ҫ���͵����ݰ�/(u8 len):����
*����ֵ:��
**************************************************************************************************************/
static void package_report_data(u8 fun,u8*data,u8 len)
{
    static u8 send_buf[40]={0};   //���static����ջ��Сһ��ѹ��
		u16 check_sum=0;
    u8 i;
		
		
		//��װЭ��ͷ
    if(len>28)return;   
    send_buf[0]=0XAA;  
		send_buf[1]=0XAA; 
    send_buf[2]=fun;    
    send_buf[3]=len; 

		//��װ����
    for(i=0;i<len;i++)send_buf[4+i]=data[i];

		//����У��ֵ
    for(i=0;i<len+4;i++)	check_sum+=send_buf[i];	
		send_buf[len+4]=((check_sum)&0xFF);
		
		//��������
		ESP32_Send_Data(send_buf,len+5);
}

/*********************************************************************************************************
*������:ANO_VER()
*���ܣ����Ͱ汾��Ϣ
*�β�:HardwareType,HardwareVER,SoftwareVER,ProtocolVER
*����ֵ:��
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
*��������ANO_MOD()
*����:���͵�ǰģʽ
*�β�:dataģʽ
*����ֵ:��
*************************************************************************************************************/
static void ANO_MOD(u8 data)
{
	u8 tbuf[1];
	tbuf[0]=(data)&0XFF;  
	package_report_data(CAR_POSE,tbuf,1);
}
/**********************************************************************************************************
*������:ANO_CAR_POSE()
*����:������̬
*�β�:angle(x����,y���,zƫ��)
*����ֵ:��
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
*������:ANO_SENSER()
*����:���ʹ�����ԭ����
*�β�:acc:������,gyro:���ٶȼ�,mag:��������
*����ֵ:��
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
*������:ANO_PID()
*����:����PID����
*�β�:PID1��PID2��PID3�Ĳ���
*����ֵ:��
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
*������:ANO_CCD_SEN()
*����:����CCD����
*�β�:ccd:CCD���ݰ���(��ֵ,��ֵ,������,������)
*����ֵ:��
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
*������:ANO_POWER()
*����:���͵���
*�β�:data:����(0%,25%,50%,75%,100%)
*����ֵ:
**************************************************************************************************************/
void ANO_POWER(u16 data)
{
	static u8 tbuf[2];
	tbuf[0]=((data)>>8)&0XFF;      
	tbuf[1]=(data)&0XFF;
	package_report_data(CAR_POWER,tbuf,2);
}
/*************************************************************************************************************
*������:ANO_MOTO()
*����:���͵��ת��
*�β�:PWM_MOTO:���ת��
*����ֵ:��
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
*������:Connect_Send_data()
*���ܣ��������ݸ���λ��
*�β�:
*����ֵ:��
**************************************************************************************************************/
void Connect_Send_data(u8 CMD_Data)
{
	
		  switch(CMD_Data)
			{
				case READ_ALL_ARG:
				//			DBG("\r�������ݵ���λ��\n");
	
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
						DBG("\r����PID����\n");
				
						ANO_PID(1);
					break;
				case READ_WORK_MODE:
					  DBG("\r���͵�ǰģʽ\n");
					  ANO_MOD(FS_MODE);
					break;
				case READ_VERINFO:
					DBG("\r���Ͱ汾��Ϣ\n");
				  
				  ANO_VER(Hardware_Type,Hardware_VER,Software_VER,Protocol_VER);
					break;
				default:
					break;
			}

}




/**************************************************************************************************************
*������:Host_Data_Receive_Anl()
*���ܣ���λ�����ݰ�����
*�β�:(u8 *data_buf):�������еĽ��յ�������/(u8 num):���ݰ�����
*����ֵ:��
**************************************************************************************************************/
void Host_Data_Receive_Anl(u8 *data_buf,u8 num)
{
	u8 sum = 0,i;
	u8 function = *(data_buf+2),cmd = *(data_buf+4);

	
	//��֡ͷ������λ�ļ���У�飻
	DBG("�յ���Ч���ݣ�");
	for(i=0;i<(num-1);i++)
	{
		
		DBG("%x  ",*(data_buf+i));
		sum += *(data_buf+i);
	}
	DBG("\r\n");
	
	////У�鲻��ֱ�ӷ���
	if(sum != *(data_buf+num-1) )		return;		
	//�ж�֡ͷ
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;	

	
	switch(function)
	{
		case MOVEMENT: 	
			DBG("����С������\n");
		
		  switch(cmd)
			{
				case CAR_STOP:
					DBG("С��ֹͣ\n");
				
					if(FS_MODE==0){
						Contrl_Turn=64;
						Movement=0;
					}
								
					break;
				case CAR_FORWARD:
					DBG("С��ǰ��\n");
				
					if(FS_MODE==0){
						Contrl_Turn=64;
						Movement=50;
					}
										
					break;
				case CAR_BACK:
					DBG("С������\n");

					if(FS_MODE==0){
						Contrl_Turn = 64;
						Movement = -50;
					}
					
					break;
				case CAR_TURN_LEFT:
					DBG("С����ת\n");

					if(FS_MODE==0){
						Contrl_Turn=30;
						Movement=0;
					}
					
					break;
				case CAR_TURN_RIGHT:
					DBG("С����ת\n");

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
			DBG("��ȡС��������Ϣ\n");

		  switch(cmd)
			{
				case READ_PID:
					DBG("��ȡPID����\n");
				
					break;
				case READ_WORK_MODE:
					DBG("��ȡ��ǰ����ģʽ\n");
				
					break;
				case READ_VERINFO:
					DBG("��ȡ�汾��Ϣ\n");
				
					break;
				case RESTORE_DEFAULT_ARG:
					DBG("�ָ�Ĭ�ϲ���\n");
				
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
			DBG("����С������ģʽ\n");
			
		  switch(cmd)
			{
				case REMOTE_MODE:
					DBG("ң��ģʽ\n");
					
					FS_MODE = 0;
				
					break;
				case AVOID_MODE:
					DBG("����ģʽ\n");
				
					FS_MODE = 1;
					break;
				case LINE_TRACK_MODE:
					DBG("Ѳ��ģʽ\n");
				
					FS_MODE = 2;
				
					break;

				default:
					break;
			}		
		
			break;
		case WRITEPID1: 
			//��λ������9����16λ����*100���͹�����,
			DBG("����С��PID1\n");
		
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
			DBG("����С��PID2\n");
		
			break;
		default:
			break;
		
	}
	
}


/**********************************************************************************************************
*������:EP32_RcvData_Extract
*����:��ȡESP32����Ч����,���ҽ��俽������������
*�β�:�жϽ��յ�����
*����ֵ:�ɹ�����0��ʧ�ܷ���1
ע�����ݸ�ʽ
							����  ֡ͷ  ֡ͷ  ����  ����  ����  У��
����          ǰ��  0xaa  0xaf  0x01  0x01  0x01  ����
***********************************************************************************************************/

u8 EP32_RcvData_Extract(const uint8_t *Buff,int len)
{	
	static u8 RxBuffer[50];
	static int data_len = 0,data_cnt = 0;
	static u8 state = 0;
	u8 i, data;
	
	
	for(i=0;i<len ;i++){
		  
		  data = Buff[i];
		
		   //�ж�֡ͷ		
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
			//�����ֽ�ֹ��0XF1
			else if(state==2&&data<0XF1)
			{
				state=3;
				RxBuffer[2]=data;
			}
			//���ݳ���С��50
				else if(state==3&&data<50)
			{
				state = 4;
				RxBuffer[3]=data;
				data_len = data;
				data_cnt = 0;
			}
			//�����ݸ��Ƶ�RxBuffer��
			else if(state==4&&data_len>0)
			{
				data_len--;
				RxBuffer[4+data_cnt++]=data;
				if(data_len==0)
					state = 5;
			}
			//����У��ֵ
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
