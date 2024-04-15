#include "math.h"
#include "stdlib.h"
#include "stm32f4xx_hal.h"
#include "contrl.h"

 



int   Dead_Zone=1200;    //�������
int   control_turn=64;                             //ת�����


//PID���ڲ���
struct pid_arg PID = {
	.Balance_Kp=200,
	.Balance_Kd=1,
	.Velocity_Kp=-52,
	.Velocity_Ki=-0.26,
	.Turn_Kp = 18,
	.Turn_Kd = 0.18,
};

/**************************************************************************************************************
*������:Read_Encoder()
*����:��ȡ������ֵ(����С����ǰǰ�����ٶ�)
*�β�:(u8 TIMX):xΪ������1����2
*����ֵ:��
*************************************************************************************************************/
int Read_Encoder(u8 TIMX)
{
    int Encoder_TIM;  
		
   switch(TIMX)
	 {
	   case 1:  Encoder_TIM= (short)TIM1 -> CNT;  TIM1 -> CNT=0;break;
		 case 2:  Encoder_TIM= (short)TIM2 -> CNT;  TIM2 -> CNT=0;break;
		 default:  Encoder_TIM=0;
	 }
		return Encoder_TIM;
}



/**************************************************************************************************************
*������:Vertical_Ring_PD()
*����:ֱ����PD����
*�β�:(float Angle):x��ĽǶ�/(float Gyro):x��Ľ��ٶ�
*����ֵ:����PIDת��֮���PWMֵ
**************************************************************************************************************/
//ֱ������PD


int	Vertical_Ring_PD(float Angle,float Gyro)
{
	 float Bias;
	 int balance;
   Bias=Angle-Mechanical_balance;
   balance=PID.Balance_Kp*Bias+ Gyro*PID.Balance_Kd;
	
	return balance;
		
	 //printf("balance = %f\n",balance);
}


/**************************************************************************************************************
*������:Vertical_speed_PI()
*���ܣ��ٶȻ�PI����
*�β�:(int encoder_left):���ֱ�����ֵ/(int encoder_right):���������ֵ�ֵ/(float Angle):x��Ƕ�ֵ
*����ֵ:
**************************************************************************************************************/

int Vertical_speed_PI(int encoder_left,int encoder_right,float Angle,float Movement )
{
	static float Velocity,Encoder_Least,Encoder;
	static float Encoder_Integral;
	Encoder_Least =(encoder_left+encoder_right)-0;    //��ȡ�����ٶ�ƫ��=�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶȣ��˴�Ϊ�㣩
	Encoder *= 0.8f;																	//һ�׵�ͨ�˲��� ���ϴε��ٶ�ռ85%
	Encoder += Encoder_Least*0.2f;                   //һ�׵�ͨ�˲����� ���ε��ٶ�ռ15% 
	Encoder_Integral +=Encoder;                       //���ֳ�λ�� ����ʱ�䣺10ms
	Encoder_Integral=Encoder_Integral-Movement; 
	
	if(Encoder_Integral>10000)  	Encoder_Integral=10000;           //�����޷�
	if(Encoder_Integral<-10000)	  Encoder_Integral=-10000;            //�����޷�

	Velocity=Encoder*PID.Velocity_Kp+Encoder_Integral*PID.Velocity_Ki;      //�ٶȿ���
	
	
	if(Turn_off(Angle)==1)   Encoder_Integral=0;            //����رպ��������
	return Velocity;
}


/**************************************************************************************************************
*������:Vertical_turn_PD()
*����:ת��PD
*�β�:��  CCDС��34��ת��CCD����64��ת�� yaw = z����������ֵ
*����ֵ:��
***************************************************************************************************************/
int Vertical_turn_PD(u8 CCD,short yaw)
{
		float Turn;     
    float Bias;	  
	  Bias=CCD-64;
	  Turn=-Bias*PID.Turn_Kp-yaw*PID.Turn_Kd;
	  return Turn;
}



/**************************************************************************************************************
*������:PWM_Limiting()
*����:PWM�޷�����
*�β�:��
*����ֵ:��
***************************************************************************************************************/
void PWM_Limiting(int *motor1,int *motor2)
{
	int Amplitude=5800;
	if(*motor1<-Amplitude) *motor1=-Amplitude;	
	if(*motor1>Amplitude)  *motor1=Amplitude;	
	if(*motor2<-Amplitude) *motor2=-Amplitude;	
	if(*motor2>Amplitude)  *motor2=Amplitude;		
}


/**************************************************************************************************************
*������:Turn_off()
*����:�رյ��
*�β�:(const float Angle):x��Ƕ�ֵ
*����ֵ:1:С����ǰ����ֹͣ״̬/0:С����ǰ��������״̬
***************************************************************************************************************/
u8 FS_state;

u8 Turn_off(const float Angle)
{
	u8 temp;
	if(fabs(Angle)>80){
		FS_state=1;
		temp=1;
		AIN2(0),			AIN1(0);
		BIN1(0),			BIN2(0);
	}
	else 
		temp=0;
		FS_state=0;
	return temp;
}

/**************************************************************************************************************
*������:Set_PWM()
*����:���PWM���Ƶ��
*�βΣ�(int motor1):���1��Ӧ��PWMֵ/(int motor2):���2��Ӧ��PWMֵ
*����ֵ:��
*************************************************************************************************************/


void Set_PWM(int motor1,int motor2)
{
	if(motor1>0)			AIN2(1),			AIN1(0);
	else 	          	AIN2(0),			AIN1(1);
	PWMA=Dead_Zone+(abs(motor1))*1.17;
	
	
	if(motor2>0)			BIN1(1),			BIN2(0);
	else       		 		BIN1(0),			BIN2(1);
	PWMB=Dead_Zone+(abs(motor2))*1.17;	
	
//	printf("PWMA = %d\n",PWMA);
//	printf("PWMB = %d\n",PWMB);
}



